# 실물 로봇 최초 구동

2026-09-07 기준. Ubuntu 24.04 / ROS 2 Jazzy, `~/ros2_ws`.

## 확정된 설정

`src/orinbot_hardware/config/hardware.yaml`에서 설정합니다.

| 항목 | 값 |
|---|---|
| 모터 | DYNAMIXEL XM430-W210, Protocol 2.0 |
| 모터 통신 속도 | **57600bps** (이전 115200bps 요청을 대체) |
| 왼쪽 / 오른쪽 ID | 1 / 2 |
| 구동륜 반지름 | 0.0325m (지름 6.5cm) |
| 좌우 바퀴 중심 간격 | 0.50m (중심에서 각각 25cm) |
| 라이다 | RPLIDAR A2M12, 256000bps |
| 카메라 | Intel RealSense D435if, 하향 15° |
| 라이다 위치 (지면의 로봇 중심 기준) | x=0.20m, y=0, z=0.52m |
| 카메라 위치 (같은 기준) | x=0.20m, y=0, z=0.45m |

57600bps는 호스트의 모터 통신 설정입니다. **모터 내부에 저장된 Baud Rate나 ID를 변경한 것은 아닙니다.** 2026-09-07 실물 시험에서 이 속도로 ID 1·2의 응답과 저속 회전을 확인했습니다. 현재 작업 범위인 전체 인터페이스 점검은 [시험 실행 안내](interface-test.md)와 [실측 결과](interface-test-results-2026-09-07.md)를 따릅니다.

## 1. 드라이버와 포트 권한 준비

현재 보드의 드라이버 설치, 계정의 `dialout` 그룹 추가, 시리얼 장치 ACL 설정은 완료했습니다. 처음 설치하거나 환경을 다시 준비할 때 사용자 터미널에서 다음을 실행합니다. sudo 비밀번호는 터미널에 입력합니다.

```bash
bash ~/ros2_ws/src/orinbot_hardware/scripts/prepare_host.sh
```

이 스크립트는 필요한 ROS 패키지를 설치하고 계정을 `dialout`에 추가하며, 현재 연결된 두 시리얼 장치에 사용자 ACL을 부여합니다. 모터 명령은 보내지 않습니다. 장치를 재연결한 뒤 권한 오류가 발생하면 로그아웃·로그인하여 그룹 변경을 적용합니다.

모든 ROS 터미널에서 환경을 불러옵니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 2. 모터 응답 확인 — PING/READ만 수행

다른 모터 제어 프로그램을 종료한 상태에서 실행합니다.

```bash
ros2 run orinbot_hardware probe_motors.py --baud 57600
```

목표 응답은 ID 1과 2, `model_number: 1030`입니다. 이 도구는 ID·통신 속도·모드·토크를 쓰지 않습니다. 응답이 없으면 전원과 TTL/RS-485 배선, 현재 저장된 속도를 확인합니다. 기존 속도를 모를 때 조회만 확장할 수 있습니다.

```bash
ros2 run orinbot_hardware probe_motors.py --baud 57600 115200 1000000
```

두 모터의 ID가 같으면 버스에서 개별 구분할 수 없습니다. 한 대씩 연결하여 DYNAMIXEL Wizard에서 왼쪽 1, 오른쪽 2와 57600bps를 설정하고 다시 조회합니다. 모델에 맞는 TTL 또는 RS-485 어댑터를 사용해야 합니다.

## 3. 토크를 끈 상태에서 ROS 연결

```bash
ros2 launch orinbot_hardware base.launch.py
```

기본값은 `enable_torque:=false`입니다. 실행 전 PING/READ로 모델, 펌웨어, 토크, 오류, 상태 응답 레벨, 워치독, 정지 상태를 검사하고 이상이 있으면 시작을 거부합니다.

이 런치는 진단 도구와 달리 **Operating Mode=1(속도 제어) 및 초기 속도·가속도·워치독 설정을 기록**합니다. ID와 Baud Rate 레지스터는 기록하지 않습니다. 토크를 켜지 않아도 컨트롤러와 엔코더 상태를 확인할 수 있습니다.

다른 터미널에서 확인합니다.

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
```

`joint_state_broadcaster`, `diff_drive_controller`가 `active`여야 합니다. `odom -> base_footprint`는 이 컨트롤러가 발행합니다. 별도 EKF가 같은 TF를 중복 발행하게 실행하지 않습니다.

## 4. 바퀴를 띄우고 첫 저속 구동

3번 런치를 Ctrl+C로 종료하고, 양쪽 구동륜을 바닥에서 띄운 상태에서 실행합니다.

```bash
ros2 launch orinbot_hardware base.launch.py enable_torque:=true
```

다른 터미널에서 키보드 조종을 시작합니다.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_footprint \
  -p speed:=0.006 -p turn:=0.020 -p repeat_rate:=10.0 -p key_timeout:=0.2 \
  -r cmd_vel:=/cmd_vel
```

`i` 전진, `,` 후진, `j`/`l` 회전, `k` 정지입니다. `TwistStamped`를 사용합니다. 최종 출력은 선속도 0.006m/s, 각속도 0.020rad/s로 제한됩니다. 직진과 회전을 합쳐도 빠른 쪽 바퀴는 0.3385rad/s로, 확인된 모터 속도 원시값 15(0.3595rad/s)보다 낮습니다. 런치는 치수와 속도 제한의 조합이 이 상한을 넘으면 시작을 거부합니다. 명령은 0.3초 후 만료되며 감속 제한을 거쳐 정지합니다. 통신 단절에는 모터 Bus Watchdog=15(300ms)를 설정하지만, 실제 정지 시간은 실물에서 추가 측정해야 합니다.

좌우 방향 초기값은 `left_direction: 1`, `right_direction: -1`입니다. 대칭 장착과 모터 Drive Mode=0을 가정하므로 실제 전진 방향을 확인해야 합니다. 방향이 틀리면 정지·런치 종료 후 해당 방향값을 바꾸고 재시험합니다. 처음부터 바닥 위에서 확인하지 않습니다.

## 5. 센서를 정지 상태에서 확인

```bash
ros2 launch orinbot_hardware sensors.launch.py
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

센서를 하나씩 확인하려면 `camera:=false` 또는 `lidar:=false`를 전달합니다. USB 2.0 연결을 고려해 영상 프로필은 640×480, 15fps로 설정했습니다. 실물 시험에서 컬러 14.50Hz, 정렬된 깊이 14.80Hz를 확인했으며 IMU는 커널 지원 부족으로 미통과입니다.

`base.launch.py`가 사용자 실측값으로 `base_footprint -> laser` 및 `base_footprint -> camera_link`를 발행합니다. 카메라 내부 광학·IMU TF는 RealSense 드라이버가 발행하여 중복을 피합니다. 센서 런치만 단독 실행하면 로봇 본체와의 TF가 없으므로, 로봇 좌표계에서 확인할 때는 베이스 런치도 실행해야 합니다. 좌우 오프셋과 라이다 방위는 중앙·전방 정렬을 가정하며 실제 설치 방향을 확인해야 합니다. 기존 시뮬레이션 Nav2 실행 파일은 실물용 시간·충돌 범위 점검이 필요합니다.

로봇 본체는 40cm 폭이지만 바퀴 중심 간격이 50cm이므로, Nav2 footprint는 타이어 바깥쪽까지 포함한 실측 외곽 크기를 사용해야 합니다.

## 50rpm 직사각형 주행

2m × 2m 경로를 한 번 도는 예제는 [직사각형 주행 실행 안내](rectangle-drive.md)를 따릅니다. 해당 런치가 베이스와 주행 코드를 함께 실행하며, 직선 최대 50rpm과 모서리 감속 설정을 적용합니다.

## 소프트웨어 검증

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select orinbot_description orinbot_hardware
source install/setup.bash
python3 -m unittest discover -s src/orinbot_hardware/test -p 'test_*.py' -v
```

모터를 사용하지 않는 통합 검증은 별도 ROS 도메인에서 실행합니다. 종료 후 테스트용 런치도 Ctrl+C로 종료합니다.

```bash
ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 launch orinbot_hardware base.launch.py mock_hardware:=true
```

```bash
ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 ~/ros2_ws/src/orinbot_hardware/test/check_mock_base.py
```

이 검증은 모의 하드웨어임을 확인한 뒤 바퀴 치수에 따른 속도 변환, 선속도·각속도 제한, 명령 중단 시 정지, 오래된 타임스탬프 거부를 검사합니다. 실물 통신·방향·주행·센서 성능을 검증한 결과는 아닙니다.

공식 자료: [ROBOTIS XM430-W210 제어표](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/), [Jazzy 다이나믹셀 하드웨어 인터페이스](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/tree/jazzy).
