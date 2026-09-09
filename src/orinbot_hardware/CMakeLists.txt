cmake_minimum_required(VERSION 3.8)
project(orinbot_hardware)
find_package(ament_cmake REQUIRED)
ament_environment_hooks(env/ros_domain.sh)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Move.srv")
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/probe_motors.py scripts/prepare_host.sh scripts/test_interfaces.py
  scripts/drive_distance.py scripts/drive_rectangle.py scripts/drive_fused_rectangle.py
  scripts/motion_service.py scripts/mapping_service.py
  scripts/keyboard_drive.py scripts/front_scan.py
  scripts/camera_preview.py
  scripts/guide_service.py scripts/guide_core.py scripts/guide_stt.py scripts/setup_guide_stt.sh
  DESTINATION lib/${PROJECT_NAME})
ament_package()
