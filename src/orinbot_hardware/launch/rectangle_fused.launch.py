"""Encoder + scan odometry + IMU rectangle test, without mapping or new robot models."""
import math
import importlib.util
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def finish_test(event, _context):
    if event.returncode:
        raise RuntimeError('Fused rectangle test failed; inspect its summary.json')
    return [EmitEvent(event=Shutdown(reason='Fused rectangle test finished'))]


def setup(context):
    share = Path(get_package_share_directory('orinbot_hardware'))
    for package in ('robot_localization', 'rtabmap_odom'):
        get_package_share_directory(package)
    width, height, rpm, duration = [float(LaunchConfiguration(k).perform(context))
                                    for k in ('width', 'height', 'rpm', 'duration')]
    turn_rpm = float(LaunchConfiguration('turn_rpm').perform(context))
    if not all(math.isfinite(v) and 0.1 <= v <= 5 for v in (width, height)):
        raise ValueError('width and height must be 0.1..5 metres')
    if not math.isfinite(rpm) or not 1 <= rpm <= 60:
        raise ValueError('rpm must be 1..60')
    if not math.isfinite(turn_rpm) or not 1 <= turn_rpm <= 25:
        raise ValueError('turn_rpm must be 1..25')
    if not math.isfinite(duration) or not 10 <= duration <= 120:
        raise ValueError('duration must be 10..120 seconds')
    # Reject an existing hardware test/driver before starting any new nodes.
    helper = Path(get_package_prefix('orinbot_hardware')) / 'lib/orinbot_hardware/test_interfaces.py'
    spec = importlib.util.spec_from_file_location('interface_preflight', helper)
    checks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checks)
    settings = yaml.safe_load((share / 'config/hardware.yaml').read_text())
    for key in ('motor_port', 'lidar_port'):
        problem = checks.port_problem(settings[key])
        if problem:
            raise RuntimeError(problem)
    if subprocess.run(['pgrep', '-f', '[r]ealsense2_camera_node|[r]ealsense-viewer'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Camera is already in use; stop the other test first')
    observe = LaunchConfiguration('observe').perform(context) == 'true'
    config = str(share / 'config/rectangle_fusion.yaml')
    follower = Node(package='orinbot_hardware', executable='drive_fused_rectangle.py',
                    output='screen', arguments=['--width', str(width), '--height', str(height),
                    '--rpm', str(rpm), '--turn-rpm', str(turn_rpm),
                    '--duration', str(duration)] + (['--observe'] if observe else []))
    icp = Node(package='rtabmap_odom', executable='icp_odometry', name='lidar_odometry',
               output='screen', parameters=[config],
               remappings=[('scan', '/scan'), ('odom', '/lidar/odom'),
                           ('odom_info', '/lidar/odom_info')])
    ekf = Node(package='robot_localization', executable='ekf_node', name='rectangle_ekf',
               output='screen', parameters=[config])
    actions = [RegisterEventHandler(OnProcessExit(target_action=follower, on_exit=finish_test))]
    # If an estimator exits, let the follower detect stale input and finish its stop/log cycle.
    actions += [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/base.launch.py')),
            launch_arguments={'enable_torque': str(not observe).lower(),
                              'rectangle_rpm': str(rpm),
                              'rectangle_turn_rpm': str(turn_rpm)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/sensors.launch.py'))),
        icp, ekf, follower,
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='1.0'),
        DeclareLaunchArgument('height', default_value='1.0'),
        DeclareLaunchArgument('rpm', default_value='60.0'),
        DeclareLaunchArgument('turn_rpm', default_value='25.0',
                              description='Turn wheel rpm, capped by rpm'),
        DeclareLaunchArgument('observe', default_value='false', choices=['true', 'false'],
                              description='Keep torque off and check fusion without driving'),
        DeclareLaunchArgument('duration', default_value='30', description='Observation duration in seconds'),
        OpaqueFunction(function=setup),
    ])
