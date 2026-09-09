cmake_minimum_required(VERSION 3.8)
project(orinbot_description)

find_package(ament_cmake REQUIRED)

# meshes 는 넣지 않습니다 — 이 패키지는 기본 도형만 쓰므로 메시 파일이 없고,
# 없는 디렉터리를 적어 두면 새로 클론한 워크스페이스에서 빌드가 실패합니다
# (git 은 빈 디렉터리를 추적하지 않습니다). 메시를 추가하면 여기에 되살리세요.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
