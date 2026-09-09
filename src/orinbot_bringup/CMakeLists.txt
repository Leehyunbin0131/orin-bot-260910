cmake_minimum_required(VERSION 3.8)
project(orinbot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

# 실기 하드웨어 드라이버 자리를 메우는 시뮬레이션 전용 노드입니다.
# battery_sim.py 는 실기에서 BMS 드라이버로 교체됩니다.
install(
  PROGRAMS
    scripts/battery_sim.py
    scripts/clock_throttle.py
    scripts/people_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
