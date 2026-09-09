#!/usr/bin/env bash
# Start the existing bounded rectangle drive and its base controller.
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
사각형 이동 테스트: 실행하면 준비 후 실제 로봇이 출발합니다.
  ./rectangle_test.sh
기본값: 2m × 2m, 직진 최대 50rpm, 왼쪽으로 회전. 중지: Ctrl+C
크기와 속도 지정:
  ./rectangle_test.sh width:=1.0 height:=1.0 rpm:=15
HELP
    exit 0
fi

test_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$test_workspace/install/setup.bash"
export ROS_DOMAIN_ID=11
exec ros2 launch orinbot_hardware rectangle.launch.py "$@"
