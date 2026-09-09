"""Real sensor drivers; base.launch.py publishes measured robot-to-sensor mounts."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    with open(LaunchConfiguration('config').perform(context)) as source:
        settings = yaml.safe_load(source)
    nodes = []
    if LaunchConfiguration('lidar').perform(context) == 'true':
        nodes.append(Node(package='sllidar_ros2', executable='sllidar_node', name='rplidar', output='screen',
                          parameters=[{'use_sim_time': False, 'channel_type': 'serial',
                                       'serial_port': settings['lidar_port'],
                                       'serial_baudrate': settings['lidar_baud_rate'],
                                       'frame_id': 'laser', 'inverted': False,
                                       'angle_compensate': True, 'scan_mode': 'Sensitivity'}]))
    if LaunchConfiguration('camera').perform(context) == 'true':
        nodes.append(Node(package='realsense2_camera', executable='realsense2_camera_node',
                          name='camera', namespace='', output='screen', parameters=[{
                              'use_sim_time': False, 'camera_name': 'camera',
                              'enable_color': True, 'enable_depth': True,
                              # Only stream the images used by this robot. The default IR
                              # profiles differ from the depth profile below.
                              'enable_infra1': False, 'enable_infra2': False,
                              'rgb_camera.color_profile': '640,480,15',
                              'depth_module.depth_profile': '640,480,15',
                              'enable_gyro': True, 'enable_accel': True, 'unite_imu_method': 2,
                              'align_depth.enable': True, 'pointcloud.enable': True,
                              # The ARM64 librealsense build exposes this filter under its NEON name.
                              'pointcloud__neon_.enable': True,
                              'publish_tf': True,
                          }]))
    return nodes


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('orinbot_hardware')) / 'config/hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
