source /opt/ros/humble/setup.bash

rtabmap_ws_local_ros="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/deps/opt/ros/humble"
if [[ -f "$rtabmap_ws_local_ros/share/ament_index/resource_index/packages/rtabmap_launch" ]]; then
    export AMENT_PREFIX_PATH="$rtabmap_ws_local_ros${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$rtabmap_ws_local_ros/lib:$rtabmap_ws_local_ros/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$rtabmap_ws_local_ros/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
    export PATH="$rtabmap_ws_local_ros/bin:$PATH"
fi
unset rtabmap_ws_local_ros
