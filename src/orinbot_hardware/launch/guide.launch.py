"""Room editor by default; drive:=true enables the real guide robot base and sensors."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,RegisterEventHandler,EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('orinbot_hardware'));drive=LaunchConfiguration('drive')
    server=Node(package='orinbot_hardware',executable='guide_service.py',output='screen',parameters=[{
        'drive':ParameterValue(drive,value_type=bool),
        'maps_directory':LaunchConfiguration('maps_directory'),'settings_file':LaunchConfiguration('settings_file'),
        'gui_port':ParameterValue(LaunchConfiguration('gui_port'),value_type=int)}])
    return LaunchDescription([
        DeclareLaunchArgument('drive',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('mock_hardware',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('lidar',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('camera',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('gui_port',default_value='8080'),
        DeclareLaunchArgument('maps_directory',default_value=str(Path.home()/'ros2_ws/maps')),
        DeclareLaunchArgument('settings_file',default_value=str(Path.home()/'.local/share/orinbot/guide.json')),
        RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[EmitEvent(event=Shutdown(reason='Guide service exited'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/base.launch.py')),condition=IfCondition(drive),
            launch_arguments={'mock_hardware':LaunchConfiguration('mock_hardware'),'enable_torque':'true','guide_mode':'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensors.launch.py')),condition=IfCondition(drive),
            launch_arguments={'lidar':LaunchConfiguration('lidar'),'camera':LaunchConfiguration('camera'),
                              'camera_preview_only':'true'}.items()),
        server])
