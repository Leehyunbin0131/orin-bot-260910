"""Real 2D mapping workspace: base + LiDAR + service manager; map starts on request."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('orinbot_hardware'))
    server = Node(package='orinbot_hardware', executable='mapping_service.py', output='screen',
                  parameters=[{'maps_directory': LaunchConfiguration('maps_directory'),
                               'gui_port': ParameterValue(LaunchConfiguration('gui_port'),value_type=int)}])
    return LaunchDescription([
        DeclareLaunchArgument('mock_hardware', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('lidar', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('maps_directory', default_value=str(Path.home()/'ros2_ws/maps')),
        DeclareLaunchArgument('gui_port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true','false']),
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[EmitEvent(event=Shutdown(reason='Mapping service exited'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/base.launch.py')),
            launch_arguments={'mock_hardware':LaunchConfiguration('mock_hardware'), 'enable_torque':'true',
                              'rectangle_rpm':'15', 'rectangle_turn_rpm':'10'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors.launch.py')),
            launch_arguments={'lidar':LaunchConfiguration('lidar'), 'camera':'false'}.items()),
        server,
        Node(package='rviz2', executable='rviz2', arguments=['-d',str(share/'config/slam.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
