# D435if RSUSB 시험 환경

현재 커널 `6.8.12-1021-tegra`에는 `CONFIG_HID_SENSOR_HUB` 지원이 없어,
기존 SDK의 native backend에서는 IMU가 비활성화됩니다. RSUSB는 USB를 통해
UVC/HID를 SDK 안에서 처리합니다. 아래 구성으로 영상과 IMU의 동시 수신을 확인했습니다.

## 별도 설치 경로

- 소스: `~/.local/src/librealsense-2.58.1-rsusb`
- 소스 버전: 공식 `v2.58.1`, 커밋 `bf2778061d5dd29776e9aca8765f75852671760b`
- 빌드: 위 소스의 `build-rsusb`
- 설치: `~/.local/opt/librealsense-2.58.1-rsusb`
- 실행 도우미: `~/ros2_ws/tools/with_rsusb.sh`

기존 ROS 패키지와 같은 SDK 버전으로 구성합니다. RSUSB와 ARM64 NEON을 켜고,
CUDA·GUI·DDS·SDK 자체 rosbag2 기록 기능은 끕니다. ROS의 `ros2 bag` 기록 기능과는
별개입니다. 기존 `/opt/ros/jazzy` SDK와 커널, 셸 기본 환경은 교체하지 않습니다.

## USB 접근 권한

RSUSB는 `/dev/bus/usb`의 카메라 장치에 읽기·쓰기 권한이 필요합니다.
2026-09-07 사용자 터미널에서 아래 명령을 실행했고, 장치의 `root:plugdev 0660`
권한 적용을 확인했습니다. 다시 실행할 필요는 없습니다. 새로 설치하는 경우 다음
명령으로 D435if(`8086:0b3a`)에만 `plugdev` 그룹의 접근 권한을 부여합니다.

```bash
sudo bash ~/ros2_ws/tools/setup_rsusb_usb_access.sh
```

이 스크립트는 카메라 USB 권한 규칙을 설치하고 해당 장치의 udev 이벤트만 갱신합니다.
SSH, 네트워크, 모터 설정을 변경하거나 시스템을 재부팅하지 않습니다.

## 장치와 스트림 확인

다른 카메라 프로그램을 종료한 상태에서 실행합니다.

```bash
bash ~/ros2_ws/tools/with_rsusb.sh rs-enumerate-devices -s
```

장치 목록에 D435if가 나타나는지 확인합니다. 가속도계·자이로를 포함한 전체
스트림 프로필은 위 명령에서 `-s`를 빼면 표시됩니다.
카메라만 ROS에서 실행하려면 다음을 사용합니다.

```bash
jazzy
bash ~/ros2_ws/tools/with_rsusb.sh ros2 launch orinbot_hardware sensors.launch.py lidar:=false
```

같은 ROS 도메인을 쓰는 다른 터미널에서 IMU와 영상 수신을 확인합니다.

```bash
jazzy
ros2 topic hz /camera/imu
```

IMU뿐 아니라 컬러·깊이 영상의 수신률, 타임스탬프, 지속 수신도 확인해야 합니다.
장치 인식만으로 전체 기능 통과를 판정하지 않습니다. 시험 종료는 카메라 실행
터미널에서 `Ctrl+C`를 사용합니다.

기존 SDK로 실행하려면 `with_rsusb.sh` 없이 기존 명령을 실행합니다.

`sensors.launch.py`는 별도 적외선 영상 출력을 `enable_infra1=false`,
`enable_infra2=false`로 명시합니다. 처음에는 기본값에 따라 IR 848×480/30fps가
깊이 640×480/15fps와 함께 열렸고, RSUSB에서 정렬 깊이 메시지가 나오지 않았습니다.
IR 영상 출력을 끈 뒤 동일한 검사에서 정렬 깊이가 수신됐습니다. 이 설정은 깊이
센서를 끄는 설정이 아닙니다.

## 카메라 자동 검사

카메라만 일정 시간 실행하고 종료하면서 컬러·깊이·IMU를 검사합니다. 모터는
접근하지 않습니다. 실행 중인 다른 카메라 프로그램이 있으면 검사를 거부합니다.

```bash
bash ~/ros2_ws/tools/with_rsusb.sh python3 ~/ros2_ws/tools/test_rsusb_camera.py --duration 30
```

로그는 `~/ros2_ws/log/rsusb_camera/<실행 시각>/`에 저장합니다. 영상은 10Hz 이상,
IMU는 20Hz 이상이어야 하고, 실제 관측 시간이 검사 시간의 절반 이상이어야 합니다.
내용 유효성·타임스탬프 증가·최근 수신 여부와 1초 미만의 메시지 간격을 함께 검사합니다.
이는 이 영상 프로필의 기본 수신 검사이며 장시간 안정성과 실제 움직임의 정확도 검증은
별도로 필요합니다.

## 실측 결과 (2026-09-07)

30초 검사와 이어진 60초 검사에서 컬러·정렬 깊이·IMU가 모두 통과했습니다.
60초 검사에는 드라이버 시작 시간이 포함되며, 실제 스트림 관측은 약 56초입니다.

| 스트림 | 수신률 | 메시지 수 | 최대 수신 간격 |
| --- | ---: | ---: | ---: |
| 컬러 640×480 | 14.84Hz | 830 | 0.204초 |
| 정렬 깊이 640×480 | 14.92Hz | 832 | 0.140초 |
| 통합 IMU | 200.06Hz | 11302 | 0.095초 |

샘플 유효성 오류와 타임스탬프 역행·중복은 없었습니다. 결과:
`~/ros2_ws/log/rsusb_camera/20260907_172220_172252/summary.json`.

시작 시 `Right MIPI error`와 USB control transfer 경고가 기록되었으나 위 수신
검사 동안 지속적인 스트림 중단은 관측되지 않았습니다. MIPI 경고는 수신 검사와
별도로 기록하며, PASS가 모든 하드웨어 경고의 소멸을 뜻하지는 않습니다.

`IMU Calibration is not available` 경고도 있습니다. SDK가 기본 보정값을 사용하므로
이번 PASS는 IMU 수신 확인이며, IMU 보정이나 정밀 위치추정 정확도의 검증이 아닙니다.

공식 안내: https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation_jetson.md

2026-09-07 추가 확인: D435if 장치만 SDK의 `hardware_reset()`으로 재초기화한 뒤
30초 재검사도 통과했고 `Right MIPI error`는 해당 실행에서 기록되지 않았습니다.
컬러 14.79Hz, 정렬 깊이 14.95Hz, IMU 200.30Hz입니다.
결과: `~/ros2_ws/log/rsusb_camera/20260907_172415_349161/summary.json`.

실행 도구 정리 후에는 `~/ros2_ws/hardware_test.sh`가 RSUSB 설정을 자동 적용합니다.
이 문서의 `tools/` 경로는 보관 전 경로이며, 현재 사용법은 워크스페이스 최상위
`README.md`를 참고합니다.
