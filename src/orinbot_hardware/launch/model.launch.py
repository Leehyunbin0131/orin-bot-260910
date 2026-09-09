"""Preview the real robot geometry without opening motors or sensor devices."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    share = Path(get_package_share_directory('orinbot_hardware'))
    spec = importlib.util.spec_from_file_location('orinbot_base_model', share / 'launch/base.launch.py')
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    settings = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    description = ET.tostring(base.build_geometry(settings), encoding='unicode')
    actions = [
        LogInfo(msg='Model preview only: no motor controller or sensor driver is started.'),
        LogInfo(msg='Caster ground frames mark swivel axes; caster wheels are shown in a fixed nominal direction.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': False}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': False}], output='screen'),
    ]
    if settings.get('wheel_width') is None:
        actions.append(LogInfo(msg='Drive wheel width is unmeasured: orange tires use provisional 3cm width, without collision geometry.'))
    if settings.get('caster_trail') is None:
        actions.append(LogInfo(msg='Caster trail is approximate: preview uses 1cm offset, without caster collision geometry.'))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        rviz = Path(get_package_share_directory('orinbot_description')) / 'rviz/orinbot.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz)],
                            parameters=[{'use_sim_time': False}], output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('orinbot_hardware'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/hardware.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
