#!/usr/bin/env bash
set -eo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
timeout 10s ros2 service call /vins_cloud_accumulator/clear std_srvs/srv/Trigger '{}'
python3 - <<'STOP_RVIZ'
import subprocess, os, signal, time
result=subprocess.run(['pgrep','-x','rviz2'],capture_output=True,text=True)
pids=[int(x) for x in result.stdout.split()]
def alive(pid):
    try:
        with open(f'/proc/{pid}/comm') as f: return f.read().strip()=='rviz2'
    except OSError: return False
for sig,wait in [(signal.SIGINT,1.5),(signal.SIGTERM,1.),(signal.SIGKILL,.2)]:
    remaining=[p for p in pids if alive(p)]
    if not remaining:break
    for pid in remaining:
        try:os.kill(pid,sig)
        except ProcessLookupError:pass
    end=time.monotonic()+wait
    while time.monotonic()<end and any(alive(p) for p in remaining):time.sleep(.05)
STOP_RVIZ
exec rviz2 -d "$WS/config/d435i/vins.rviz"
