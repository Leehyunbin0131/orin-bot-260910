#!/usr/bin/env bash
# Use the separate RSUSB SDK for this command and its child processes.
set -eo pipefail

rsusb_prefix=/home/hyunlee/.local/opt/librealsense-2.58.1-rsusb
if [[ $# == 0 || ${1:-} == --help ]]; then
    echo 'Usage: bash ~/ros2_ws/tools/with_rsusb.sh COMMAND [ARGUMENTS...]'
    echo 'Inspect camera: bash ~/ros2_ws/tools/with_rsusb.sh rs-enumerate-devices -s'
    echo 'Start camera: bash ~/ros2_ws/tools/with_rsusb.sh ros2 launch orinbot_hardware sensors.launch.py lidar:=false'
    exit 0
fi
[[ -r "$rsusb_prefix/lib/librealsense2.so.2.58" ]] || {
    echo "RSUSB SDK is not installed yet: $rsusb_prefix" >&2
    exit 1
}
source /opt/ros/jazzy/setup.bash
source /home/hyunlee/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH="$rsusb_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$rsusb_prefix/bin:$PATH"
exec "$@"
