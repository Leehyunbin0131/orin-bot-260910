#!/usr/bin/env bash
set -eo pipefail
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
센서 융합 사각형 테스트: 엔코더 + 라이다 이동 추정 + 카메라 IMU + EKF
  ./rectangle_fused_test.sh
기본값: 1m × 1m, 직진 최대 60rpm, 회전 바퀴 최대 25rpm. 준비 후 실제 주행합니다. 중지: Ctrl+C
토크를 끈 채 센서 융합만 30초 검사:
  ./rectangle_fused_test.sh observe:=true
크기·속도 변경: width:=1.0 height:=1.0 rpm:=60 turn_rpm:=25
회전 바퀴 속도는 rpm 상한도 함께 적용합니다.
HELP
    exit 0
fi
test_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rsusb_prefix=/home/hyunlee/.local/opt/librealsense-2.58.1-rsusb
[[ -r "$rsusb_prefix/lib/librealsense2.so.2.58" ]] || {
    echo "RSUSB SDK를 찾을 수 없습니다: $rsusb_prefix" >&2
    exit 1
}
source /opt/ros/jazzy/setup.bash
source "$test_workspace/install/setup.bash"
export LD_LIBRARY_PATH="$rsusb_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$rsusb_prefix/bin:$PATH"
export ROS_DOMAIN_ID=11
exec ros2 launch orinbot_hardware rectangle_fused.launch.py "$@"
