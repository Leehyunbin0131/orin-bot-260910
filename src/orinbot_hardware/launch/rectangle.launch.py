"""One command for the bounded rectangle example; mock mode never opens motors."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math


def setup(context):
    share = Path(get_package_share_directory('orinbot_hardware'))
    mock = LaunchConfiguration('mock_hardware').perform(context) == 'true'
    rpm = float(LaunchConfiguration('rpm').perform(context))
    width = float(LaunchConfiguration('width').perform(context))
    height = float(LaunchConfiguration('height').perform(context))
    if not math.isfinite(rpm) or not 1 <= rpm <= 50:
        raise ValueError('rpm must be 1..50')
    if not all(math.isfinite(v) and 0.1 <= v <= 5 for v in (width, height)):
        raise ValueError('width and height must be 0.1..5 metres')
    follower = Node(package='orinbot_hardware', executable='drive_rectangle.py', output='screen',
                    arguments=['--width', str(width), '--height', str(height), '--rpm', str(rpm)]
                    + (['--mock'] if mock else []))
    return [
        RegisterEventHandler(OnProcessExit(target_action=follower, on_exit=[
            EmitEvent(event=Shutdown(reason='Rectangle program finished'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/base.launch.py')),
                                 launch_arguments={'mock_hardware': str(mock).lower(),
                                                   'enable_torque': 'true', 'rectangle_rpm': str(rpm)}.items()),
        follower,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='2.0'),
        DeclareLaunchArgument('height', default_value='2.0'),
        DeclareLaunchArgument('rpm', default_value='50.0'),
        DeclareLaunchArgument('mock_hardware', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
