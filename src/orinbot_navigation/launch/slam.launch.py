"""RTAB-Map RGB-D VSLAM launch file.

    ros2 launch orinbot_navigation slam.launch.py                    # SLAM mapping mode
    ros2 launch orinbot_navigation slam.launch.py localization:=true  # Localization only mode

TF Structure:
    map --(rtabmap)--> vodom --(rgbd_odometry)--> odom --(EKF)--> base_footprint
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Sensor topic configuration (realsense2_camera standard)
GRAY_TOPIC = '/camera/color/image_raw'
INFO_TOPIC = '/camera/color/camera_info'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'

# IMU topic configuration
RAW_IMU_TOPIC = '/camera/imu'
IMU_TOPIC = '/camera/imu/data'

BASE_FRAME = 'base_footprint'
WHEEL_ODOM_FRAME = 'odom'
VISUAL_ODOM_FRAME = 'vodom'

ODOM_TOPIC_FOR_RTABMAP = PythonExpression([
    "'/vodom' if '", LaunchConfiguration('use_vslam'), "' == 'true' "
    "else '/odometry/filtered'"])

RTABMAP_REMAPPINGS = [
    ('rgb/image', GRAY_TOPIC),
    ('rgb/camera_info', INFO_TOPIC),
    ('depth/image', DEPTH_TOPIC),
    ('odom', ODOM_TOPIC_FOR_RTABMAP),
    ('scan', '/scan'),
    ('imu', IMU_TOPIC),
]


def generate_launch_description():
    localization = LaunchConfiguration('localization')

    args = [
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='If true, perform localization only using existing database'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/orinbot_rtabmap.db',
            description='RTAB-Map database path'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false',
            description='RTAB-Map visualization window'),
        DeclareLaunchArgument(
            'reg_strategy', default_value='2',
            description='Loop closure strategy (0=Vis, 1=ICP, 2=Vis+ICP)'),
        DeclareLaunchArgument(
            'memory_thr', default_value='3000',
            description='Max working memory node count'),
        DeclareLaunchArgument(
            'detection_rate', default_value='2.0',
            description='RTAB-Map detection rate [Hz]'),
        DeclareLaunchArgument(
            'use_vslam', default_value='true',
            description='Use visual odometry (rgbd_odometry)'),
        DeclareLaunchArgument(
            'use_imu_in_odom', default_value='false',
            description='Use IMU in visual odometry'),
        DeclareLaunchArgument(
            'map_3d', default_value='false',
            description='Generate 3D occupancy grid and cloud map'),
        DeclareLaunchArgument(
            'use_lidar_in_slam', default_value='true',
            description='Input LiDAR scan to RTAB-Map'),
    ]

    # Visual Odometry
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': BASE_FRAME,
            'odom_frame_id': VISUAL_ODOM_FRAME,
            'guess_frame_id': WHEEL_ODOM_FRAME,
            'publish_tf': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            'wait_for_transform': 0.3,
            'publish_null_when_lost': False,
            'wait_imu_to_init': LaunchConfiguration('use_imu_in_odom'),
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
            'Odom/GuessMotion': 'true',
            'OdomF2M/MaxSize': '1000',
            'Vis/MaxFeatures': '1000',
            'Vis/MinInliers': '15',
            'Vis/EstimationType': '1',
            'Reg/Force3DoF': 'true',
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('use_vslam')),
    )

    # Visual Odometry TF relay (50 Hz)
    vodom_tf_relay = Node(
        package='orinbot_navigation',
        executable='vodom_tf_relay.py',
        name='vodom_tf_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'visual_odom_topic': '/vodom',
            'visual_odom_frame': VISUAL_ODOM_FRAME,
            'wheel_odom_frame': WHEEL_ODOM_FRAME,
            'base_frame': BASE_FRAME,
            'publish_rate': 50.0,
            'tf_tolerance': 0.2,
        }],
        condition=IfCondition(LaunchConfiguration('use_vslam')),
    )

    # SLAM (Mapping + Loop Closure)
    rtabmap_parameters = {
        'use_sim_time': True,
        'frame_id': BASE_FRAME,
        'map_frame_id': 'map',
        'odom_frame_id': '',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'publish_tf': True,
        'database_path': LaunchConfiguration('database_path'),
        'wait_for_transform': 0.3,
        'tf_delay': 0.05,
        'tf_tolerance': 0.2,
        'Optimizer/GravitySigma': '0.0',
        'Reg/Force3DoF': 'true',
        'RGBD/OptimizeMaxError': '3.0',
        'Rtabmap/MemoryThr': ParameterValue(
            LaunchConfiguration('memory_thr'), value_type=str),
        'Rtabmap/DetectionRate': ParameterValue(
            LaunchConfiguration('detection_rate'), value_type=str),
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/ProximityBySpace': 'true',
        'Reg/Strategy': ParameterValue(
            LaunchConfiguration('reg_strategy'), value_type=str),
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/PointToPlane': 'false',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/CorrespondenceRatio': '0.3',
        'Grid/Sensor': '2',
        'Grid/3D': ParameterValue(LaunchConfiguration('map_3d'), value_type=str),
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '5.0',
        'Grid/RayTracing': 'true',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.08',
        'Grid/MaxObstacleHeight': '1.0',
        'GridGlobal/MinSize': '10.0',
    }

    # Mapping mode node
    rtabmap_mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, {'Mem/IncrementalMemory': 'true'}],
        remappings=RTABMAP_REMAPPINGS,
        arguments=['--delete_db_on_start'],
        condition=UnlessCondition(localization),
    )

    # Localization mode node
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters, {
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(localization),
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': BASE_FRAME,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
        }],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    return LaunchDescription(args + [
        rgbd_odometry,
        vodom_tf_relay,
        rtabmap_mapping,
        rtabmap_localization,
        rtabmap_viz,
    ])
