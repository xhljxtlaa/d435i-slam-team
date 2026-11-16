#!/usr/bin/env bash
# No ROS processes or camera access. Reuse the regression tests from deployment.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TMP_CHECK="$(mktemp -d)"
trap 'rm -rf "$TMP_CHECK"' EXIT
export TMP_CHECK
python3 - <<'PY'
import ast
import importlib.util
import json
import os
import subprocess
from pathlib import Path

root = Path.cwd()
listed = subprocess.run(['git', 'ls-files', '-z'], text=True, capture_output=True)
paths = listed.stdout.split('\0') if listed.returncode == 0 else []
if not any(paths):
    paths = [str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and '.git' not in p.parts]
counts = {'shell': 0, 'python': 0, 'json': 0}
for name in filter(None, paths):
    p = root / name
    if p.suffix == '.sh':
        subprocess.run(['bash', '-n', str(p)], check=True)
        counts['shell'] += 1
    elif p.suffix == '.py':
        ast.parse(p.read_text(), filename=str(p))
        counts['python'] += 1
    elif p.suffix == '.json':
        json.loads(p.read_text())
        counts['json'] += 1
print('PASS syntax and JSON:', counts)
spec = importlib.util.spec_from_file_location('prepare_runtime', root / 'vins_ws/tools/prepare_runtime.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
run = Path(os.environ['TMP_CHECK']) / 'run config with spaces'
config = module.prepare(root / 'vins_ws', run)
text = config.read_text()
assert '/home/he/' not in text
assert str(run) in text
assert (run / 'left.yaml').is_file() and (run / 'right.yaml').is_file()
assert (run / 'pose_graph').is_dir()
print('PASS VINS runtime configuration relocation')
PY
g++ -std=c++14 -Wall -Wextra -pedantic orb3_ws/tools/test_imu_buffer.cpp -o "$TMP_CHECK/imu"
"$TMP_CHECK/imu"
g++ -std=c++14 -Wall -Wextra -pedantic orb3_ws/tools/test_trajectory_continuity.cpp -o "$TMP_CHECK/trajectory"
"$TMP_CHECK/trajectory"
read -r -a CV_FLAGS <<< "$(pkg-config --cflags --libs opencv4)"
g++ -std=c++14 -Wall -Wextra -pedantic orb3_ws/tools/test_cloud_geometry.cpp "${CV_FLAGS[@]}" -o "$TMP_CHECK/cloud"
"$TMP_CHECK/cloud"
VINS_SOURCE="$ROOT/vins_ws/src/VINS-Fusion-ROS2-Humble/vins/src"
for NAME in depth_observation tracking_health; do
  g++ -std=c++14 -Wall -Wextra -pedantic -I/usr/include/eigen3 -I"$VINS_SOURCE" \
    "vins_ws/tests/${NAME}_test.cpp" "${CV_FLAGS[@]}" -o "$TMP_CHECK/$NAME"
  "$TMP_CHECK/$NAME"
done
echo 'PASS: offline checks complete; hardware and full ROS builds require separate validation.'
