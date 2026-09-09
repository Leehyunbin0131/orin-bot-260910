"""Saved-map localization and navigation. Velocity output goes through the guide gate."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('orinbot_hardware'));params=str(share/'config/guide_nav.yaml')
    servers=[('nav2_map_server','map_server',[{'yaml_filename':LaunchConfiguration('map')}]),
             ('nav2_amcl','amcl',[]),('nav2_planner','planner_server',[]),
             ('nav2_controller','controller_server',[]),
             ('nav2_bt_navigator','bt_navigator',[{'default_nav_to_pose_bt_xml':str(share/'config/guide_bt.xml')}])]
    nodes=[Node(package=pkg,executable=name,name=name,parameters=[params,*extra],output='screen',
                remappings=[('cmd_vel','/guide/nav_cmd_vel')]) for pkg,name,extra in servers]
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='guide_lifecycle',
        parameters=[{'autostart':True,'use_sim_time':False,'node_names':[name for _,name,_ in servers]}],output='screen'))
    return LaunchDescription([DeclareLaunchArgument('map'),*nodes])
