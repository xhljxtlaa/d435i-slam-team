#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace/setup.bash"
exec ros2 launch "$workspace/d435i.launch.py" "$@"
