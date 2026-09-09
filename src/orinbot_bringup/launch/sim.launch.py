"""Gazebo simulator and robot bridge/controller integrated launch file.

    ros2 launch orinbot_bringup sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('orinbot_description')
    bringup_share = FindPackageShare('orinbot_bringup')

    xacro_file = PathJoinSubstitution([desc_share, 'urdf', 'orinbot.urdf.xacro'])
    controllers_file = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])
    bridge_config = PathJoinSubstitution([bringup_share, 'config', 'gz_bridge.yaml'])
    rviz_config = PathJoinSubstitution([bringup_share, 'config', 'sim.rviz'])
    world_file = PathJoinSubstitution([bringup_share, 'worlds', LaunchConfiguration('world')])

    # Launch arguments
    args = [
        DeclareLaunchArgument('world', default_value='room.sdf',
                              description='World file name in worlds/'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI (headless if false)'),
        DeclareLaunchArgument('use_pointcloud', default_value='true',
                              description='Generate pointcloud using depth_image_proc'),
        DeclareLaunchArgument('clock_rate', default_value='100.0',
                              description='Publish rate for ROS /clock [Hz] (0 for original passthrough)'),
    ]

    # robot_description
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=true',
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Gazebo simulation resource path registration
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([bringup_share, 'models']),
    )

    headless_flag = PythonExpression([
        "'' if '", LaunchConfiguration('gui'), "' == 'true' else '-s '"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', headless_flag, world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'orinbot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ROS <-> Gazebo transport bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    # Depth pointcloud generation container
    pointcloud_container = ComposableNodeContainer(
        name='camera_pointcloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image_rect', '/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/depth/camera_info'),
                    ('points', '/camera/depth/points'),
                ],
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='depth_register',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('depth/camera_info', '/camera/depth/camera_info'),
                    ('depth/image_rect', '/camera/depth/image_rect_raw'),
                    ('depth_registered/camera_info',
                     '/camera/aligned_depth_to_color/camera_info'),
                    ('depth_registered/image_rect',
                     '/camera/aligned_depth_to_color/image_raw'),
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('use_pointcloud')),
    )

    # Downsample /clock_raw to /clock
    clock_throttle = Node(
        package='orinbot_bringup',
        executable='clock_throttle.py',
        name='clock_throttle',
        output='screen',
        parameters=[{'rate': LaunchConfiguration('clock_rate'),
                     'use_sim_time': False}],
    )

    # Controller spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-ros-args', '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
            '--controller-ros-args', '-r /diff_drive_controller/odom:=/odom',
        ],
    )

    # Sequential event handlers: spawn -> JSB -> diff_drive
    spawn_then_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )
    jsb_then_diff_drive = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[diff_drive_spawner])
    )

    # EKF node (wheel odometry + IMU fusion)
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[PathJoinSubstitution([bringup_share, 'config', 'ekf.yaml'])],
    )

    diff_drive_then_ekf = RegisterEventHandler(
        OnProcessExit(target_action=diff_drive_spawner, on_exit=[ekf])
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        clock_throttle,
        spawn_robot,
        gz_bridge,
        pointcloud_container,
        spawn_then_jsb,
        jsb_then_diff_drive,
        diff_drive_then_ekf,
        rviz,
    ])
