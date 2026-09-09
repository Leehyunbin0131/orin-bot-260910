"""도킹만 떼어 낸 시험대. 여러 개를 병렬로 띄우기 위한 것입니다.

    ros2 launch orinbot_navigation dock_bench.launch.py x:=1.02 y:=-3.10 yaw:=-1.5708

SLAM 도 Nav2 도 띄우지 않습니다. 도킹 정확도를 재는 데 필요한 것은
시뮬레이터 + 카메라 + 마커 검출 + staged_dock 뿐이고, 나머지는 인스턴스당
2~3 코어를 더 먹어 병렬 수를 그만큼 깎습니다.

빠지는 것 때문에 달라지는 동작 두 가지 (둘 다 경고만 남기고 넘어갑니다):
  - `_goto_entry` 는 Nav2 가 필요하므로 쓰지 않습니다. 시험 스크립트가
    `navigate_to_staging_pose: false` 로 목표를 걸고, 로봇을 처음부터
    정렬 지점 근처에 스폰합니다.
  - `_face_dock` / `_restore_standoff` / SLAM 동결은 map 프레임과
    rtabmap 서비스가 필요합니다. 없으면 건너뜁니다.

`twist_mux` 도 없으므로 staged_dock 의 출력을 /cmd_vel 로 바로 보냅니다.

병렬 실행은 인스턴스마다 ROS_DOMAIN_ID 와 GZ_PARTITION 을 다르게 주어
격리합니다 (실측: 각 도메인이 /clock 발행자를 1개씩만 봅니다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare('orinbot_navigation')
    bringup_share = FindPackageShare('orinbot_bringup')
    params_file = LaunchConfiguration('params_file')
    common = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([nav_share, 'config', 'docking.yaml'])),
        DeclareLaunchArgument('world', default_value='room.sdf'),
        # 로봇 스폰 자세. 케이스마다 다르게 주어 초기 오차를 만듭니다.
        DeclareLaunchArgument('x', default_value='1.0'),
        DeclareLaunchArgument('y', default_value='-3.10'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        DeclareLaunchArgument('initial_soc', default_value='0.85'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, 'launch', 'sim.launch.py'])]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': 'false',
            'use_rviz': 'false',
            # 코스트맵을 쓰지 않으므로 포인트클라우드 합성을 끕니다.
            'use_pointcloud': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    detector = Node(
        package='orinbot_navigation', executable='dock_marker_board.py',
        name='dock_marker_board', output='screen',
        parameters=[params_file, common, {'lock_distance': 0.0}],
        remappings=[('image', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info'),
                    ('detected_dock_pose', '/detected_dock_pose')],
    )

    dock = Node(
        package='orinbot_navigation', executable='staged_dock.py',
        name='staged_dock', output='screen',
        parameters=[params_file, common],
        # twist_mux 가 없으므로 바로 컨트롤러로 보냅니다.
        remappings=[('cmd_vel_dock', '/cmd_vel')],
    )

    battery = Node(
        package='orinbot_bringup', executable='battery_sim.py',
        name='battery_sim', output='screen',
        parameters=[common, {'initial_soc': LaunchConfiguration('initial_soc')}],
    )

    return LaunchDescription(args + [sim, detector, dock, battery])
