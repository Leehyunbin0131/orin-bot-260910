# ROS 주행 제어 실물 검증

`tools/ros_base_test.py`는 자체적으로 베이스 런치를 실행하여 `/cmd_vel` → `diff_drive_controller` → Dynamixel → `/joint_states` → `/odom` 경로를 검사합니다. 센서 드라이버·SLAM·Nav2는 실행하지 않습니다. 다른 모터 제어 프로그램과 베이스 런치를 종료한 상태에서 실행합니다.

```bash
jazzy
cd ~/ros2_ws
# 모터를 사용하지 않는 ROS 통합 시험
python3 tools/ros_base_test.py --mode mock
# 실물 드라이버 기동, 토크 OFF 상태의 엔코더·오도메트리·TF 확인
python3 tools/ros_base_test.py --mode stationary
# 바퀴를 띄워 고정한 뒤 실물 저속 회전 시험
python3 tools/ros_base_test.py --mode motion --wheels-lifted
```

펌웨어 업데이트처럼 USB를 다시 연결하면 기존 장치의 임시 ACL이 사라질 수 있습니다. 이 보드의 계정은 이미 `dialout`에 등록되어 있습니다. 이전부터 열려 있던 터미널에서 시리얼 접근 권한 오류가 나면 `newgrp dialout`을 실행하고, 새 셸에서 `jazzy`로 ROS 환경을 다시 불러옵니다.

기본 모드는 `stationary`입니다. 이 모드도 베이스 드라이버가 속도 제어 모드·가속도·워치독 등을 설정합니다. SDK의 PING/READ 전용 시험과는 다릅니다. 종료 시 토크 OFF와 목표 속도 0을 확인하고, 시험 전 운영 모드·가속도·워치독을 복구합니다. 운영 모드 복구가 목표 속도 레지스터를 초기화하므로 복구 후 다시 0을 기록하고 조회합니다. 운영 모드 변경으로 초기화되는 다른 RAM 튜닝 값까지 모두 복구하는 도구는 아닙니다.

실물 회전 시험은 전진, 후진, 좌회전, 우회전, 직진+회전을 각 2초씩 명령합니다. 단계 사이에 명령 발행을 멈춰 실제 엔코더 속도와 오도메트리 속도가 정지하는 시간을 측정합니다. 이어서 2초 지난 타임스탬프의 명령을 발행하여 회전하지 않는지 확인합니다. 바퀴를 띄운 상태이므로 거리는 바퀴 엔코더 기준이며 차체의 실제 이동거리를 검증하지 않습니다.

속도 제한은 선속도 ±0.006m/s, 각속도 ±0.020rad/s입니다. 조합 시 최대 바퀴 속도는 0.3385rad/s(모터 원시값 약 14.12)입니다. 모의 시험에서만 상한보다 큰 입력을 보내 제한 동작을 검증하며, 실물에는 위 저속 명령을 사용합니다.

전용 도메인 기본값은 177이고 로컬 호스트로 탐색을 제한합니다. 이미 토픽이 발행되는 도메인이거나 모터 포트를 다른 프로세스가 사용하면 시작을 거부합니다. 피드백 중단·통신/하드웨어 오류·예상 속도 초과·정지 실패가 감지되면 시험을 중단하고 종료 처리를 수행합니다. 종료 처리는 직접 기동한 프로세스 그룹의 잔여 자식도 정리한 뒤 시리얼 포트로 최종 상태를 확인합니다.

로그는 `~/ros2_ws/log/ros_base_tests/<실행시각>/`에 저장합니다.

- `summary.json`: 판정, 실제 적용 파라미터, 명령 중단 후 정지 시간, 엔코더와 오도메트리 비교, 수신률.
- `events.jsonl`: 명령·관절 위치/속도·오도메트리·모터 오류·종료 상태의 시간별 기록.
- `base.log`: ROS 드라이버와 컨트롤러 출력.
- `hardware.yaml`, `controllers.yaml`, `robot_description.urdf`: 실행 구성.

## 2026-09-07 초기 점검에서 수정한 사항

오른쪽 모터 펌웨어 44는 Fast Sync Read를 지원하지 않습니다. 설치된 Dynamixel 드라이버 1.5.2는 초기 10회 실패 후 일반 Sync Read로 전환하지만, 기존 시작 제한 200ms가 먼저 만료되어 기동에 실패했습니다. 임시로 제한을 1000ms로 늘려 일반 읽기 방식의 정지 상태 시험에 통과했습니다. 이후 사용자가 펌웨어를 업데이트했고, 양쪽 버전 50·ID 1/2·57600bps를 확인했습니다. 최종 구성은 오류 제한을 200ms로 복구하고, ROS 베이스 사전 검사에서 펌웨어 45 이상을 요구합니다. 업데이트 후 Fast Sync Read 오류 없이 실제 회전 시험까지 통과했습니다.

모의 하드웨어는 `calculate_dynamics=true`로 속도를 위치에 적분하도록 설정했습니다. 이 설정이 없으면 속도 명령은 보이지만 위치 기반 오도메트리가 0에 머무릅니다.

설치된 Dynamixel 드라이버 1.5.2의 `/dynamixel_hardware_interface/dxl_state` 중 `torque_state`는 실제 활성화 상태를 반영하지 않았습니다. 이 필드를 토크 판정에 사용하지 않습니다. GPIO 상태 인터페이스에 `Torque Enable`, `Goal Velocity`를 추가하여 매 읽기 주기마다 실제 레지스터를 수신하고, `/dynamic_joint_states`의 `left_motor`, `right_motor` 항목으로 검사합니다. `Torque Enable`은 0/1, `Goal Velocity`는 rad/s입니다. 통신·하드웨어 오류도 함께 기록합니다.

제조사 근거: [Fast Sync Read 지원 펌웨어](https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a), [ROS 모의 하드웨어 적분 설정](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html#component-parameters).

## 검증 상태

- [업데이트 후 최종 실물 정지 상태 시험](../log/ros_base_tests/20260907_151819_028426/summary.json): 통과.
- [ROS 모의 회전·명령 중단·타임스탬프 시험](../log/ros_base_tests/20260907_150951_977453/summary.json): 통과.
- [실물 ROS 회전 시험](../log/ros_base_tests/20260907_151906_594683/summary.json): 통과.
- [실측 결과 보고서](ros-base-test-results-2026-09-07.md).

## 60rpm 전진 시험

사용자의 60rpm 요청을 반영한 별도 실행 옵션입니다. 바퀴를 띄워 고정한 상태에서 실행합니다.

```bash
jazzy
cd ~/ros2_ws
python3 tools/ros_base_test.py --mode motion --wheels-lifted --rpm-sweep 60
```

양쪽 바퀴를 전진 방향으로 15→30→45→60rpm 순서로 각 3초씩 명령하고, 각 단계 마지막 1초의 속도 중앙값으로 추종 여부를 검사합니다. 이후 속도 0을 명령하고 감속 정지·토크 OFF를 확인합니다. 작은 목표값도 `--rpm-sweep 30`처럼 지정할 수 있으며 최대 60rpm입니다.

이 실행은 ROS 런치에 `straight_test_rpm`을 전달하여 해당 실행의 직진 상한을 높입니다. 회전 명령은 0으로 제한하고 선형 가감속을 0.05m/s²로 설정합니다. 60rpm은 원시값 262(명목 59.998rpm), 바퀴 외주 속도 약 0.204m/s입니다. 원래의 저속 종합 시험은 `--rpm-sweep` 없이 실행합니다.

실물 시험에서 왼쪽 59.97rpm, 오른쪽 59.85rpm을 확인했으며, 속도 0 명령 후 약 4.27초에 정지 판정을 통과했습니다. [60rpm 실측 결과와 로그](motor-60rpm-results-2026-09-07.md).
