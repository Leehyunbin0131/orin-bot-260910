cmake_minimum_required(VERSION 3.8)
project(orinbot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/vodom_tf_relay.py
    scripts/frontier_explorer.py
    scripts/nav2_startup_watchdog.py
    scripts/auto_dock.py
    scripts/dock_marker_board.py
    scripts/staged_dock.py
    scripts/dock_register.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
