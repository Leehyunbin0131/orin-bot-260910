"""Saved-map localization and navigation. Velocity output goes through the guide gate."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share=Path(get_package_share_directory('orinbot_hardware'));params=str(share/'config/guide_nav.yaml')
    settings=yaml.safe_load((share/'config/hardware.yaml').read_text())
    linear=262*.0239691227*settings['wheel_radius']
    angular=2*linear/settings['wheel_separation']
    speed_parameters={'FollowPath.desired_linear_vel':linear,
                      'FollowPath.rotate_to_heading_angular_vel':angular,
                      'FollowPath.max_angular_accel':angular/4.0}
    servers=[('nav2_map_server','map_server',[{'yaml_filename':LaunchConfiguration('map')}]),
             ('nav2_amcl','amcl',[]),('nav2_planner','planner_server',[]),
             ('nav2_controller','controller_server',[speed_parameters]),
             ('nav2_bt_navigator','bt_navigator',[{'default_nav_to_pose_bt_xml':str(share/'config/guide_bt.xml')}])]
    nodes=[Node(package=pkg,executable=name,name=name,parameters=[params,*extra],output='screen',
                remappings=[('cmd_vel','/guide/nav_cmd_vel')]) for pkg,name,extra in servers]
    # Costmap activation needs map -> odom. Let the user set an initial pose
    # without consuming the planner's activation timeout first.
    for name, names, autostart in (
        ('guide_localization', ['map_server','amcl'], True),
        ('guide_lifecycle', ['planner_server','controller_server','bt_navigator'], False)):
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name=name,
            parameters=[{'autostart':autostart,'use_sim_time':False,'node_names':names}],output='screen'))
    return LaunchDescription([DeclareLaunchArgument('map'),*nodes])
