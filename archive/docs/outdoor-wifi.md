# 맥북으로 야외에서 SSH 접속

로봇이 `ORINBOT` 와이파이 핫스팟을 만들고 맥북이 여기에 접속합니다. 로봇에는 모니터나 추가 공유기가 필요하지 않습니다. 이 로봇의 `wlP1p1s0` 무선 장치는 AP 모드를 지원하며, NetworkManager와 DHCP용 dnsmasq가 설치되어 있습니다.

## 1. 실내에서 한 번 준비

로봇에 SSH로 접속한 현재 터미널에서 실행합니다. 네트워크 설정 파일을 만들기 때문에 `sudo`가 필요하며, 비밀번호는 터미널에 입력합니다.

```bash
sudo bash ~/ros2_ws/tools/robot_wifi.sh setup
```

이 단계는 현재 와이파이를 전환하지 않습니다. 출력되는 **ORINBOT 와이파이 비밀번호를 맥북에 저장**합니다. 임의로 생성된 비밀번호이며 SSH 로그인 비밀번호와는 별개입니다.

## 2. 핫스팟으로 전환

로봇이 정지한 상태에서 먼저 실내에서 연결을 확인합니다.

```bash
sudo orinbot-wifi outdoor
```

3초 후 로봇 와이파이가 전환되어 기존 SSH 연결이 끊길 수 있습니다. 맥북의 와이파이 메뉴에서 `ORINBOT`을 선택하고 저장한 비밀번호를 입력합니다. 맥북의 새 터미널에서 접속합니다.

```bash
ssh hyunlee@10.42.0.1
```

새 IP로 처음 접속할 때 SSH 호스트 키 확인이 나올 수 있습니다. 전환 전에 로봇에서 `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`로 확인한 지문과 대조할 수 있습니다.

핫스팟 활성화 후 **3분 안에 새 주소로 SSH 연결이 감지되면 야외 모드가 저장**됩니다. 이후 전원을 다시 켜도 핫스팟이 자동 연결 후보 중 우선순위 100으로 선택됩니다. 현재 실내 와이파이 프로필의 우선순위는 0입니다. SSH 연결이 감지되지 않으면 이전 실내 와이파이로 복구를 시도합니다. 실내 와이파이에 도달할 수 있는 장소에서 최초 전환을 진행합니다.

연결을 확인한 뒤에는 맥북과 로봇을 함께 밖으로 가져가 같은 주소로 접속합니다. 와이파이 신호가 닿는 범위에서 사용할 수 있으며, 실제 야외 통신 거리는 현장에서 확인해야 합니다. 별도 인터넷 연결이 없더라도 SSH와 로봇 내부 ROS 프로그램은 사용할 수 있습니다.

## 3. 주행과 실내 복귀

로봇 SSH 터미널에서 [직사각형 주행](rectangle-drive.md)을 실행합니다. 이 명령은 실제 주행을 시작합니다.

```bash
jazzy
ros2 launch orinbot_hardware rectangle.launch.py
```

실내로 돌아와 주행을 종료한 뒤 기존 와이파이로 되돌리려면 다음을 실행합니다.

```bash
sudo orinbot-wifi indoor
```

맥북도 실내 와이파이에 연결하고 기존 로봇 주소 또는 `ssh hyunlee@orin.local`로 다시 접속합니다. 실내 연결에 실패하면 핫스팟으로 복귀를 시도합니다.

비밀번호 확인과 전환 로그 조회:

```bash
sudo orinbot-wifi info
sudo journalctl -u orinbot-wifi-switch -n 50 --no-pager
```

설정은 `/etc/NetworkManager/system-connections/orinbot-outdoor.nmconnection`, 기존 와이파이 식별자는 `/var/lib/orinbot-wifi/indoor.uuid`에 저장합니다. 기존 실내 프로필의 비밀번호나 설정은 바꾸지 않습니다.

현재 검증 범위는 무선 장치의 AP 지원, 필수 프로그램 설치 여부, 스크립트의 구문과 설정 형식입니다. 실제 핫스팟 활성화와 맥북 접속은 위 절차로 확인해야 합니다.

공식 자료: [NetworkManager 핫스팟](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli.html), [고정 주소를 사용하는 연결 공유](https://www.networkmanager.dev/docs/api/latest/settings-ipv4.html), [자동 연결 우선순위](https://www.networkmanager.dev/docs/api/latest/settings-connection.html).
