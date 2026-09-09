"""Start the encoder-based incremental move service and its bounded base controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('orinbot_hardware'))
    mock = LaunchConfiguration('mock_hardware')
    server = Node(package='orinbot_hardware', executable='motion_service.py', output='screen',
                  parameters=[{'mock_hardware': ParameterValue(mock, value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='false', choices=['true', 'false']),
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            EmitEvent(event=Shutdown(reason='Motion service exited'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/base.launch.py')),
            launch_arguments={'mock_hardware': mock, 'enable_torque': 'true',
                              'rectangle_rpm': '10', 'rectangle_turn_rpm': '10'}.items()),
        server,
    ])
