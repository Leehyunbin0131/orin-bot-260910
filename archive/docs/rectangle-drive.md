# 50rpm으로 2m × 2m 주행

`drive_rectangle.py`는 **2m 전진 → 정지 → 왼쪽으로 90도 제자리 회전**을 네 번 반복합니다. 마지막 회전까지 마치면 시작 방향으로 정지합니다. 직선에서는 바퀴 속도를 최대 50rpm(현재 바퀴 반지름 기준 약 0.17m/s)으로 올리고, 모서리에서는 약 10rpm으로 회전합니다. 출발과 정지 구간에는 가감속이 적용됩니다.

## 실행

바닥 주행을 하려면 띄워 둔 바퀴를 내려놓고, 로봇 전방과 왼쪽에 주행 공간을 확보합니다. 2m × 2m는 로봇 중심의 목표 경로이므로 본체와 바퀴가 차지하는 공간도 필요합니다. 기존 베이스 런치와 키보드 조종 프로그램은 종료합니다.

```bash
jazzy
ros2 launch orinbot_hardware rectangle.launch.py
```

**이 명령은 모터 토크를 켜고 준비가 끝나면 자동으로 출발합니다.** 베이스 제어도 함께 실행하므로 별도로 `base.launch.py`를 실행하지 않습니다. 한 바퀴에 약 2분이 걸리며, 완료 시 정지하고 런치가 종료됩니다. 중간에 멈추려면 실행 터미널에서 `Ctrl+C`를 누릅니다.

크기나 직진 속도를 바꾸려면 다음처럼 지정합니다. 길이는 0.1~5m, 속도는 1~50rpm 범위입니다.

```bash
ros2 launch orinbot_hardware rectangle.launch.py width:=2.0 height:=2.0 rpm:=50
```

모의 실행은 선택 사항입니다. 아래 명령은 모터 포트를 열지 않습니다.

```bash
ros2 launch orinbot_hardware rectangle.launch.py mock_hardware:=true
```

`jazzy`가 없는 기존 터미널에서는 `source ~/.bashrc`를 한 번 실행합니다. 시리얼 포트 권한 오류가 나면, 현재 터미널에서 아래 순서로 이미 등록된 `dialout` 그룹을 적용하고 재실행합니다. 마지막 명령은 실제 주행을 시작합니다.

```bash
newgrp dialout
jazzy
ros2 launch orinbot_hardware rectangle.launch.py
```

`newgrp dialout`은 새 셸을 열어 그룹 권한을 적용하므로 한 줄씩 입력합니다. 로그아웃 후 다시 로그인하면 이후 터미널에도 적용되어 `newgrp`를 반복할 필요가 없습니다. 패키지를 다시 설치할 필요는 없습니다.

## 동작 기준과 로그

거리와 각도는 `/odom`의 바퀴 엔코더 기반 추정값으로 판단합니다. 정지 판단 오차 범위는 거리 1.5cm, 각도 약 0.86도입니다. 실제 바닥에서는 타이어 미끄러짐과 바퀴 치수 오차 때문에 경로가 달라질 수 있습니다. **라이다·카메라를 사용하는 장애물 회피는 포함하지 않습니다.**

방향 보정 명령을 포함해 각 바퀴의 목표 속도를 50rpm 이내로 제한합니다. 오도메트리 수신이 0.5초 이상 끊기거나, 5초 동안 진행이 없거나, 다른 `/cmd_vel` 발행자가 발견되면 중단합니다.

실행마다 다음 위치에 로그를 저장하며, 터미널에도 해당 경로를 출력합니다.

```text
~/ros2_ws/log/rectangle_runs/<실행 시각>/
  events.jsonl  # 위치, 각도, 명령 속도, 구간별 진행 기록
  summary.json  # 완료 여부, 네 모서리 위치, 최종 정지 확인
```

2026-09-07 검증: 전체 2m × 2m 모의 주행과 소프트웨어 테스트 24개를 통과했습니다. 실제 바닥 경로 정확도는 아직 측정하지 않았습니다.

- [모의 주행 결과](../log/rectangle_runs/20260907_154247_919563/summary.json)
- [모의 실행 출력](../log/rectangle_runs/20260907_154247_919563/launch.log)
- [소프트웨어 테스트 결과](../log/rectangle-software-verification-20260907.txt)
- [주행 코드](../src/orinbot_hardware/scripts/drive_rectangle.py)
- [통합 실행 파일](../src/orinbot_hardware/launch/rectangle.launch.py)
