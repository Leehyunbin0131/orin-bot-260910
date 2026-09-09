"""Nav2 autonomous navigation stack launch file.

    ros2 launch orinbot_navigation nav2.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Lifecycle nodes to be configured/activated in sequence by lifecycle_manager
LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    twist_mux_file = LaunchConfiguration('twist_mux_file')

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'nav2_params.yaml']),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'twist_mux_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'config', 'twist_mux.yaml']),
            description='twist_mux parameter file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate lifecycle nodes'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='Group 6 servers into a single process (experimental)'),
        DeclareLaunchArgument(
            'startup_watchdog', default_value='true',
            description='Automatically retry lifecycle activation if it fails'),
        DeclareLaunchArgument(
            'bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('orinbot_navigation'), 'behavior_trees',
                'navigate_w_fast_recovery.xml']),
            description='navigate_to_pose behavior tree XML file'),
    ]

    common = [params_file, {'use_sim_time': True}]
    bt_extra = {'default_nav_to_pose_bt_xml': LaunchConfiguration('bt_xml')}
    composed = IfCondition(LaunchConfiguration('use_composition'))
    separate = UnlessCondition(LaunchConfiguration('use_composition'))

    # (package, executable, component plugin, node name, remappings, extra parameters)
    CMD_VEL_NAV = [('cmd_vel', 'cmd_vel_nav')]
    SERVERS = [
        ('nav2_controller', 'controller_server',
         'nav2_controller::ControllerServer', 'controller_server', CMD_VEL_NAV, None),
        ('nav2_planner', 'planner_server',
         'nav2_planner::PlannerServer', 'planner_server', [], None),
        ('nav2_behaviors', 'behavior_server',
         'behavior_server::BehaviorServer', 'behavior_server', CMD_VEL_NAV, None),
        ('nav2_bt_navigator', 'bt_navigator',
         'nav2_bt_navigator::BtNavigator', 'bt_navigator', [], bt_extra),
        ('nav2_velocity_smoother', 'velocity_smoother',
         'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
         CMD_VEL_NAV, None),
    ]

    lifecycle_params = [{
        'use_sim_time': True,
        'autostart': LaunchConfiguration('autostart'),
        'node_names': LIFECYCLE_NODES,
    }]

    # Separate process launch configuration
    separate_nodes = [
        Node(package=pkg, executable=exe, name=name, output='screen',
             parameters=common + ([extra] if extra else []),
             remappings=remap, condition=separate)
        for pkg, exe, _plugin, name, remap, extra in SERVERS
    ] + [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=lifecycle_params, condition=separate),
    ]

    # Composed container launch configuration
    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg, plugin=plugin, name=name,
                parameters=common + ([extra] if extra else []), remappings=remap,
                extra_arguments=[{'use_intra_process_comms': True}])
            for pkg, _exe, plugin, name, remap, extra in SERVERS
        ] + [
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=lifecycle_params,
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        condition=composed,
    )

    # Multiplex velocity commands by priority
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_file, {'use_sim_time': True}],
        remappings=[('cmd_vel_out', 'cmd_vel')],
    )

    # Lifecycle startup watchdog node
    watchdog = Node(
        package='orinbot_navigation',
        executable='nav2_startup_watchdog.py',
        name='nav2_startup_watchdog',
        output='screen',
        parameters=[],
        condition=IfCondition(LaunchConfiguration('startup_watchdog')),
    )

    return LaunchDescription(
        args + separate_nodes + [container, twist_mux, watchdog])
