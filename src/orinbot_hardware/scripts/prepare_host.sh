#!/usr/bin/env bash
# 사용자 터미널에서 실행: sudo 비밀번호를 채팅에 입력하지 않습니다.
set -euo pipefail
if [[ $(id -u) == 0 ]]; then
  echo '일반 사용자로 실행하세요. 필요한 명령만 sudo로 실행합니다.' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-dynamixel-sdk \
  ros-jazzy-dynamixel-hardware-interface ros-jazzy-realsense2-camera \
  ros-jazzy-teleop-twist-keyboard python3-serial python3-numpy acl git build-essential
sudo usermod -aG dialout "$(id -un)"
# 현재 세션도 사용 가능하게, 이 로봇의 두 장치에만 사용자 ACL을 부여합니다.
# 재연결 시 그룹 변경 적용을 위해 재로그인해야 합니다.
for device in \
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBIN9M3-if00-port0 \
  /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_3915548a17a0164cb877f2ab2ade5994-if00-port0; do
  if [[ -e "$device" ]]; then
    sudo setfacl -m "u:$(id -un):rw" "$(readlink -f "$device")"
  fi
done
# The released legacy rplidar_ros driver crashed on this A2M12/ARM64 setup.
# Build the verified upstream ROS 2 driver without replacing system packages.
orinbot_workspace="${ORINBOT_WORKSPACE:-${HOME}/ros2_ws}"
orinbot_lidar_source="${orinbot_workspace}/src/sllidar_ros2"
if [[ ! -d "${orinbot_lidar_source}" ]]; then
  git clone --no-checkout https://github.com/Slamtec/sllidar_ros2.git "${orinbot_lidar_source}"
  git -C "${orinbot_lidar_source}" checkout --detach 34300099fadfc772965962dec837bf436706188f
fi
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "${orinbot_workspace}/install/setup.bash" ]]; then
  source "${orinbot_workspace}/install/setup.bash"
fi
set -u
cd "${orinbot_workspace}"
colcon build --symlink-install --packages-up-to orinbot_hardware
echo '드라이버 설치·빌드 및 현재 포트 권한 설정 완료. 모터 구동 명령은 전송하지 않았습니다.'
