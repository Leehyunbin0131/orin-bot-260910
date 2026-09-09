#!/usr/bin/env bash
# Hardware interface test with the installed RSUSB camera backend.
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
하드웨어 테스트: 라이다·카메라(컬러/깊이/IMU)·모터 통신을 검사합니다.
  ./hardware_test.sh
바퀴를 띄워 고정한 뒤, 짧은 모터 회전 검사도 포함하려면:
  ./hardware_test.sh --motion --wheels-lifted
추가 옵션: --duration 30 (10~120초)
HELP
    exit 0
fi

test_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rsusb_prefix=/home/hyunlee/.local/opt/librealsense-2.58.1-rsusb
if [[ ! -r "$rsusb_prefix/lib/librealsense2.so.2.58" ]]; then
    echo "RSUSB SDK를 찾을 수 없습니다: $rsusb_prefix" >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$test_workspace/install/setup.bash"
export LD_LIBRARY_PATH="$rsusb_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$rsusb_prefix/bin:$PATH"
export ROS_DOMAIN_ID=11
exec ros2 run orinbot_hardware test_interfaces.py "$@"
