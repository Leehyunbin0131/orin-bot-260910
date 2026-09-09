"""Charging docking integrated launch file (Marker detection + docking_server + battery simulation + auto dock).

    ros2 launch orinbot_navigation docking.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'docking.yaml']),
            description='Docking parameter file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate docking_server'),
        DeclareLaunchArgument(
            'startup_watchdog', default_value='true',
            description='Automatically retry lifecycle activation if failed'),
        DeclareLaunchArgument(
            'use_battery_sim', default_value='true',
            description='Battery simulator toggle'),
        DeclareLaunchArgument(
            'battery_speedup', default_value='1.0',
            description='Discharge and charge speedup multiplier'),
        DeclareLaunchArgument(
            'initial_soc', default_value='0.85',
            description='Initial battery state of charge (0 to 1)'),
        DeclareLaunchArgument(
            'auto_dock', default_value='true',
            description='Automatically return to dock when battery is low'),
        DeclareLaunchArgument(
            'docking_mode', default_value='staged',
            description="'staged' for step-by-step docking (staged_dock.py), 'smooth' for opennav_docking"),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/color/image_raw',
            description='Camera image topic for marker detection'),
        DeclareLaunchArgument(
            'dock_register', default_value='true',
            description='Register the dock pose when booting while charging'),
    ]

    common = {'use_sim_time': use_sim_time}

    detector = Node(
        package='orinbot_navigation',
        executable='dock_marker_board.py',
        name='dock_marker_board',
        output='screen',
        parameters=[params_file, common, {
            'lock_distance': ParameterValue(PythonExpression([
                "0.0 if '", LaunchConfiguration('docking_mode'),
                "' == 'staged' else 0.45"]), value_type=float),
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic')),
            ('camera_info', '/camera/color/camera_info'),
            ('detected_dock_pose', '/detected_dock_pose'),
        ],
    )

    staged = LaunchConfiguration('docking_mode')
    is_staged = PythonExpression(["'", staged, "' == 'staged'"])
    is_smooth = PythonExpression(["'", staged, "' != 'staged'"])

    staged_server = Node(
        package='orinbot_navigation',
        executable='staged_dock.py',
        name='staged_dock',
        output='screen',
        parameters=[params_file, common],
        condition=IfCondition(is_staged),
    )

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file, common],
        remappings=[('cmd_vel', 'cmd_vel_dock')],
        condition=IfCondition(is_smooth),
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['docking_server'],
        }],
        condition=IfCondition(is_smooth),
    )

    watchdog = Node(
        package='orinbot_navigation',
        executable='nav2_startup_watchdog.py',
        name='docking_startup_watchdog',
        output='screen',
        parameters=[{'manager': '/lifecycle_manager_docking'}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('startup_watchdog'),
            "' == 'true' and '", staged, "' != 'staged'"])),
    )

    battery = Node(
        package='orinbot_bringup',
        executable='battery_sim.py',
        name='battery_sim',
        output='screen',
        parameters=[common, {
            'speedup': LaunchConfiguration('battery_speedup'),
            'initial_soc': LaunchConfiguration('initial_soc'),
        }],
        condition=IfCondition(LaunchConfiguration('use_battery_sim')),
    )

    supervisor = Node(
        package='orinbot_navigation',
        executable='auto_dock.py',
        name='auto_dock',
        output='screen',
        parameters=[params_file, common],
        condition=IfCondition(LaunchConfiguration('auto_dock')),
    )

    # 도크에 붙은 채 부팅하면 그 자세를 도크 좌표로 등록합니다.
    # 스테이션을 옮겨도 로봇을 한 번 밀어 넣고 재부팅하면 끝입니다.
    registrar = Node(
        package='orinbot_navigation',
        executable='dock_register.py',
        name='dock_register',
        output='screen',
        parameters=[params_file, common],
        condition=IfCondition(LaunchConfiguration('dock_register')),
    )

    return LaunchDescription(
        args + [detector, staged_server, docking_server, lifecycle,
                watchdog, battery, supervisor, registrar])
