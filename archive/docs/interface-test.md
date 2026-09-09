# 라이다·카메라·모터 통합 점검과 로그

현재 목적은 최초 하드웨어 점검입니다. SLAM/Nav2는 실행하지 않습니다.
다른 베이스·센서 드라이버와 모터 제어 프로그램은 종료하고 실행합니다.

## 준비

2026-09-07 현재 보드의 드라이버 설치와 시리얼 포트 접근 권한 설정은 완료했습니다. 처음 설치하거나 환경을 다시 준비할 때는 사용자 터미널에서 다음을 실행하고 sudo 비밀번호를 입력합니다.

```bash
bash ~/ros2_ws/src/orinbot_hardware/scripts/prepare_host.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

설치가 완료된 환경에서는 위의 두 `source` 명령으로 ROS 환경만 불러오면 됩니다. 실물 시험 결과와 원본 로그는 [2026-09-07 결과 보고서](interface-test-results-2026-09-07.md)에 정리했습니다.

## 회전 없이 전체 인터페이스 확인

```bash
ros2 run orinbot_hardware test_interfaces.py
```

- 라이다: `/scan` 메시지 수·수신률, 정상 거리 샘플 수, 최소/최대 거리, 타임스탬프 증가.
- 카메라: 컬러 영상 크기·형식·수신률, 정렬된 깊이 영상의 유효값 비율과 최소/중앙/최대 거리, IMU 수신과 유한값 여부. 포인트클라우드는 보조 확인 항목입니다.
- 모터: 57600bps, ID 1·2에 PING/READ. 모델·펌웨어·토크·모드·워치독·엔코더·전류 원시값·전압·온도·통신 지연을 기록합니다. 이 모드에서는 모터 설정을 쓰지 않습니다.

센서는 기본 30초 관측합니다. 다른 길이가 필요하면 `--duration 60`처럼 지정합니다(10~120초).
센서 점검이 끝나면 드라이버를 종료하고 모터를 점검합니다. 특정 인터페이스가 실패해도 나머지 결과를 수집합니다.
센서 전용 ROS 도메인은 기본 176입니다. 이미 사용하는 도메인이면 `--domain`으로 다른 값을 지정합니다.

## 바퀴 회전 시험 추가

**양쪽 구동륜을 바닥에서 띄워 고정한 다음** 실행합니다.

```bash
ros2 run orinbot_hardware test_interfaces.py --motion --wheels-lifted
```

순서는 좌륜 1초 → 정지 → 우륜 1초 → 정지 → 양륜 전진 1초 → 정지 → 양륜 후진 1초 → 정지입니다.
목표 속도는 기본 원시값 15(약 3.44rpm), 바퀴 외주 속도 약 0.012m/s입니다.
고속에서 동작하지 않는다는 사용자 설명을 반영하여 CLI에서도 원시값 15를 상한으로 제한합니다. 낮추려면 `--velocity-raw 10`처럼 지정합니다.
명령 부호에 맞는 엔코더 변화와 정지 후 속도를 검사합니다. 모터 전류값은 원시값으로 기록합니다.

`--motion`은 두 모터가 모두 정지·토크 OFF·정상 모델인 것을 확인한 뒤 속도 제어 모드(Operating Mode=1), 초기 목표 속도 0, 가속도, Bus Watchdog=15(300ms)를 설정하고 토크를 켭니다. Operating Mode 변경은 EEPROM 기록입니다. ID와 통신 속도 레지스터는 바꾸지 않습니다.

시험 종료·예외·Ctrl+C 시 양쪽 토크 OFF 및 목표 속도 0을 시도하고 확인합니다. 통신이 정상일 때 기존 Operating Mode, Profile Acceleration, Bus Watchdog를 복구합니다. **토크는 OFF로 남겨 둡니다.** 운영 모드 변경 시 펌웨어가 다른 RAM 제어값도 초기화할 수 있으므로 기존 튜닝 상태를 완전히 보존하는 시험은 아닙니다.

정지 확인 실패도 로그에 기록합니다. 이 경우 전원을 차단하고 배선·전원을 확인해야 합니다. 모터 응답만으로 실제 장착 방향이나 바닥에서의 주행 성능을 판정하지 않으므로, 바퀴가 기대한 방향으로 도는지도 직접 확인합니다.

## 엔코더 기준 1m 전진 회전

바퀴를 띄워 고정한 상태에서 양쪽 바퀴를 1m에 해당하는 만큼 전진 회전시키려면 다음을 실행합니다.

```bash
ros2 run orinbot_hardware drive_distance.py --distance 1.0 --velocity-raw 15 --placement wheels-lifted
```

이 상태에서는 로봇 본체가 이동하지 않습니다. 거리값은 지름 6.5cm와 4096펄스/회전 엔코더로 계산합니다. 약 86초가 필요하며 목표 근처에서는 더 감속합니다. 좌우 엔코더 거리를 비교해 앞선 바퀴 속도를 낮추고, 원시 속도 15를 넘기지 않습니다. 3초 이상 전진이 없거나 좌우 거리 차이가 3cm를 넘으면 중단합니다. 종료 시 목표 속도 0과 토크 OFF를 확인합니다.

실제로 바닥에서 이동할 때는 전방 공간을 확보하고 로봇을 내려놓은 것을 확인한 뒤 `--placement floor-clear`를 사용합니다. 이 시험은 센서 기반 장애물 회피나 외부 위치 측정을 수행하지 않습니다.

거리 시험 로그는 별도로 `~/ros2_ws/log/distance_tests/<실행시각>/`에 저장합니다. `events.jsonl`에는 좌우·평균 거리와 모터 상태, `summary.json`에는 최종 거리와 정지 결과가 있습니다.

## 인터페이스 점검 로그 위치와 판정

실행할 때마다 새 폴더를 만들고 경로를 출력합니다.

```text
~/ros2_ws/log/interface_tests/YYYYMMDD_HHMMSS_ffffff/
  summary.txt       사람이 읽는 전체 결과
  summary.json      인터페이스별 판정, 수신률, 모터 단계 결과
  events.jsonl      시간별 센서 요약·모터 측정·오류·정지 확인
  hardware.yaml     시험에 사용한 설정 사본
  usb.txt           USB 장치 목록
  usb_tree.txt      USB 연결 구조·전송 속도
  camera.log        카메라 드라이버 출력 (기동했을 때)
  lidar.log         라이다 드라이버 출력 (기동했을 때)
```

원본 영상이나 전체 포인트클라우드를 저장하는 rosbag 녹화는 하지 않습니다. 진단용 통계·상태 로그를 저장합니다.
`PASS`는 필수 스트림 수신·데이터 유효성 또는 모터 응답 검사를 통과했다는 뜻입니다. 회전 시험 여부는 `motion_requested`, `motion_tested`, `phases`에 별도로 기록합니다. 명령 종료 코드는 모두 통과하면 0, 하나라도 실패하면 1입니다.

초기 점검에서 `미설치`, `시리얼 접근 권한 없음`은 실행 환경 문제이며 하드웨어 고장 판정이 아닙니다.

설정: `src/orinbot_hardware/config/hardware.yaml`.
테스트 코드: `src/orinbot_hardware/scripts/test_interfaces.py`.
ROS `/cmd_vel`과 오도메트리를 포함한 별도 통합 시험은 [ROS 주행 제어 검증](ros-base-test.md)을 참고합니다.
소프트웨어 회귀 검증:

```bash
cd ~/ros2_ws
python3 -m unittest discover -s src/orinbot_hardware/test -p 'test_*.py' -v
```

공식 제어표: [XM430-W210](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/).

## 2026-09-07 실기 확인 후 드라이버 변경

- 배포판 `rplidar_ros` 2.1.0의 `rplidar_composition`이 스캔 시작 후 `malloc(): invalid size (unsorted)`로 종료되어 제조사 `sllidar_ros2`를 워크스페이스에서 빌드하여 사용합니다. 소스는 `src/sllidar_ros2`, 확인한 커밋은 `34300099fadfc772965962dec837bf436706188f`입니다. 설치 스크립트도 같은 구성으로 갱신했습니다.
- 현재 ARM64 librealsense 2.58.1의 포인트클라우드 필터는 ROS 파라미터 이름이 `pointcloud__neon_.enable`입니다. 기존 `pointcloud.enable`과 함께 이 파라미터도 설정하여 현재 보드에서 활성화합니다.
- 커널 `6.8.12-1021-tegra`에서 `CONFIG_HID_SENSOR_HUB`가 비활성화되어 카메라 HID가 `hid-generic`에 연결되고 IIO 장치가 생성되지 않습니다. 카메라 로그에도 `No HID info provided, IMU is disabled`가 기록됩니다. 따라서 IMU는 필수 항목 실패로 유지하며 전체 카메라 PASS로 처리하지 않습니다.

제조사 자료: [SLAMTEC ROS 2 드라이버](https://github.com/Slamtec/sllidar_ros2), [RealSense Jetson 백엔드·커널 지원 안내](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation_jetson.md).
