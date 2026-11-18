#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ "${1:-}" == --help ]]; then
  echo '用法：scripts/record_bag.sh [秒数，默认20] [采集场景说明]'
  echo '先启动 D435i 驱动。保存同步红外、深度、IMU、CameraInfo、TF 与匹配配置。'
  exit 0
fi
DURATION="${1:-20}"
NOTE="${2:-未提供场景或运动说明}"
[[ "$DURATION" =~ ^[1-9][0-9]*$ ]] || { echo '秒数必须是正整数'; exit 1; }
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$ROOT/data/recordings"
SESSION="$(mktemp -d "$ROOT/data/recordings/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
mkdir -p "$SESSION/config/vins"
cp "$ROOT"/vins_ws/config/d435i/{left.yaml,right.yaml,depth_imu.yaml,camera.yaml} "$SESSION/config/vins/"
# Requires live camera messages/TF/device_info; refuses inconsistent geometry.
PROFILE=()
if [[ "${ORB3_INTRINSICS:-vins}" == vins ]]; then
  PROFILE=(--intrinsics-profile "$ROOT/orb3_ws/config/vins_intrinsics.json")
elif [[ "${ORB3_INTRINSICS}" != driver ]]; then
  echo 'ORB3_INTRINSICS 必须为 vins 或 driver'; exit 1
fi
python3 "$ROOT/orb3_ws/tools/capture_config.py" --output "$SESSION/config/orb3_camera.yaml" "${PROFILE[@]}"
python3 - "$SESSION" "$DURATION" "$NOTE" <<'PY'
import datetime,json,subprocess,sys
from pathlib import Path
p=Path(sys.argv[1])
metadata={'created_at':datetime.datetime.now().astimezone().isoformat(),
          'requested_seconds':int(sys.argv[2]),'scene_and_motion':sys.argv[3],
          'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
          'status':'recording_not_validated','ground_truth_available':False}
(p/'recording.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
PY
echo "采集目录：$SESSION"
set +e
timeout --signal=INT --kill-after=15s "${DURATION}s" ros2 bag record -s sqlite3 \
  --compression-mode message --compression-format zstd --compression-threads 2 \
  --max-bag-size 268435456 --max-cache-size 67108864 -o "$SESSION/bag" \
  /camera/camera/infra1/image_rect_raw /camera/camera/depth/image_rect_raw \
  /camera/camera/imu /camera/camera/infra1/camera_info /camera/camera/depth/camera_info \
  /tf /tf_static
STATUS=$?
set -e
[[ "$STATUS" == 0 || "$STATUS" == 124 ]] || { echo '录制异常退出，保留目录供检查'; exit "$STATUS"; }
python3 - "$SESSION" <<'PY'
import json,sys,yaml
from pathlib import Path
p=Path(sys.argv[1]);metadata=yaml.safe_load((p/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
counts={entry['topic_metadata']['name']:entry['message_count'] for entry in metadata['topics_with_message_count']}
required=['/camera/camera/infra1/image_rect_raw','/camera/camera/depth/image_rect_raw','/camera/camera/imu',
          '/camera/camera/infra1/camera_info','/camera/camera/depth/camera_info','/tf_static']
missing=[topic for topic in required if counts.get(topic,0)==0]
record=json.loads((p/'recording.json').read_text())
record.update(status='missing_topics' if missing else 'topic_counts_verified',topic_message_counts=counts,missing_topics=missing)
(p/'recording.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
if missing:raise SystemExit('采集不完整：'+', '.join(missing))
print('必需话题均有消息；仍需人工检查同步、运动与回放效果。')
PY
ros2 bag info "$SESSION/bag"
echo "采集完成：$SESSION"
