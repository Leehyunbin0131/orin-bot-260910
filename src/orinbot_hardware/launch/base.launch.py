"""Real XM430 wheel base. Defaults to torque OFF; mock_hardware uses no serial I/O."""
from pathlib import Path
import math
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def validate_settings(settings):
    for key in ('wheel_radius', 'wheel_separation', 'profile_size', 'body_length', 'body_width',
                'body_height', 'ground_clearance', 'caster_separation',
                'caster_radius', 'caster_width', 'caster_height',
                'caster_plate_length', 'caster_plate_width'):
        value = float(settings[key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if any(settings[key] <= 2*settings['profile_size'] for key in ('body_length', 'body_width', 'body_height')):
        raise ValueError('Frame dimensions must exceed two profile widths')
    for key in ('drive_axle_x', 'caster_x'):
        if not math.isfinite(float(settings[key])):
            raise ValueError(f'{key} must be finite')
    if settings['caster_x'] >= settings['drive_axle_x']:
        raise ValueError('Rear caster axes must be behind the drive axle')
    for key in ('wheel_width', 'caster_trail'):
        value = settings.get(key)
        if value is not None and (not math.isfinite(float(value)) or float(value) <= 0):
            raise ValueError(f'{key} must be positive or null when unmeasured')
    if settings['caster_height'] > settings['ground_clearance']:
        raise ValueError('Caster mounting plate would be above the body bottom; check mounting geometry')
    if 2 * settings['caster_radius'] > settings['caster_height']:
        raise ValueError('Caster wheel diameter exceeds total caster height')
    for side in ('left', 'right'):
        motor_id = settings[f'{side}_id']
        if type(motor_id) is not int or not 0 <= motor_id <= 252:
            raise ValueError(f'{side}_id must be an integer from 0 to 252')
        if settings[f'{side}_direction'] not in (-1, 1):
            raise ValueError(f'{side}_direction must be -1 or 1')
    if settings['left_id'] == settings['right_id']:
        raise ValueError('Left and right motor IDs must differ')
    if settings['baud_rate'] not in (9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000):
        raise ValueError('Unsupported XM430 baud rate')
    for sensor in ('lidar', 'camera'):
        for axis in ('x', 'y', 'z'):
            key = f'{sensor}_{axis}'
            if not math.isfinite(float(settings[key])):
                raise ValueError(f'{key} must be finite')
        if float(settings[f'{sensor}_z']) <= 0:
            raise ValueError(f'{sensor}_z must be above the floor')
    for key in ('camera_pitch_degrees', 'lidar_yaw_degrees'):
        if not math.isfinite(float(settings[key])):
            raise ValueError(f'{key} must be finite')


def verify_bus(settings):
    """Reject incorrect IDs, moving/enabled motors and faults before any configuration write."""
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    path = settings['motor_port']
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(
            f'Cannot access {path}; if your account is already in dialout, '
            "run 'newgrp dialout', then 'jazzy' and retry (or log out and log in). "
            'Otherwise run prepare_host.sh in your terminal; also check the USB connection.')
    port, packet = PortHandler(path), PacketHandler(2.0)
    try:
        if not port.openPort() or not port.setBaudRate(settings['baud_rate']):
            raise RuntimeError('Could not open motor port at configured baud rate')
        for motor_id in (settings['left_id'], settings['right_id']):
            model, result, error = packet.ping(port, motor_id)
            if result != COMM_SUCCESS:
                raise RuntimeError(f'ID {motor_id}: communication failed (result={result}); '
                                   'check motor power/cable/baud and run probe_motors.py')
            if model != 1030:
                raise RuntimeError(f'ID {motor_id}: XM430-W210 not confirmed '
                                   f'(model={model}, packet_error=0x{error:02x}); run probe_motors.py')
            if error:
                detail = f'packet_error=0x{error:02x}'
                if error == 0x80:
                    status, read_result, read_error = packet.read1ByteTxRx(port, motor_id, 70)
                    if read_result == COMM_SUCCESS and not (read_error & 0x7f):
                        detail += f', hardware_error={status}'
                raise RuntimeError(f'ID {motor_id}: motor fault ({detail}); '
                                   'inspect motor with probe_motors.py before starting')
            for address, name in ((6, 'firmware'), (64, 'torque'), (68, 'return_level'),
                                  (70, 'hardware_error'), (98, 'watchdog')):
                value, result, error = packet.read1ByteTxRx(port, motor_id, address)
                if result != COMM_SUCCESS or error:
                    raise RuntimeError(f'ID {motor_id}: cannot read {name}')
                # Fast Sync Read requires X430 firmware 45 or later.
                bad = ((name == 'firmware' and value < 45)
                       or (name in ('torque', 'hardware_error') and value != 0)
                       or (name == 'watchdog' and value == 255)
                       or (name == 'return_level' and value != 2))
                if bad:
                    raise RuntimeError(f'ID {motor_id}: {name}={value}; inspect motor before starting')
            velocity, result, error = packet.read4ByteTxRx(port, motor_id, 128)
            if result != COMM_SUCCESS or error or velocity != 0:
                raise RuntimeError(f'ID {motor_id}: wheel is moving or velocity read failed')
    finally:
        port.closePort()


def validate_controller_limits(settings, parameters, max_wheel_raw=15):
    """Reject limits that can exceed the wheel speed confirmed on this robot."""
    limits = []
    for axis in ('linear.x', 'angular.z'):
        if parameters.get(f'{axis}.has_velocity_limits') is not True:
            raise ValueError(f'{axis}: velocity limits must be enabled')
        upper, lower = (float(parameters[f'{axis}.{key}_velocity']) for key in ('max', 'min'))
        if not (math.isfinite(upper) and math.isfinite(lower) and lower <= 0 <= upper):
            raise ValueError(f'{axis}: finite positive/negative velocity limits required')
        limits.append(max(upper, -lower))
    wheel_speed = (limits[0] + limits[1] * settings['wheel_separation'] / 2) / settings['wheel_radius']
    if not any(limits) or wheel_speed > max_wheel_raw * 0.0239691227 + 1e-9:
        raise ValueError(f'Combined linear/angular limits exceed XM430 raw velocity {max_wheel_raw}')


def configure_straight_speed_test(settings, parameters, rpm):
    """Explicit, bounded straight-only test profile; normal launch is unchanged."""
    if not math.isfinite(rpm) or not 1 <= rpm <= 60:
        raise ValueError('Straight speed test must be between 1 and 60 rpm')
    raw = math.floor(rpm / 0.229)
    speed = raw * 0.0239691227 * settings['wheel_radius']
    parameters.update({'linear.x.max_velocity': speed, 'linear.x.min_velocity': -speed,
                       'angular.z.max_velocity': 0.0, 'angular.z.min_velocity': 0.0,
                       'linear.x.max_acceleration': 0.05, 'linear.x.max_deceleration': -0.05})
    validate_controller_limits(settings, parameters, raw)
    return raw


def configure_rectangle_profile(settings, parameters, rpm, turn_rpm=10.0):
    """Set axis limits; the rectangle follower bounds combined wheel commands to rpm."""
    if not math.isfinite(rpm) or not 1 <= rpm <= 60:
        raise ValueError('Rectangle speed must be between 1 and 60 rpm')
    if not math.isfinite(turn_rpm) or not 1 <= turn_rpm <= 25:
        raise ValueError('Rectangle turn speed must be between 1 and 25 rpm')
    turn_rpm = min(turn_rpm, rpm)
    response_scale = max(1.0, turn_rpm/10.0)
    linear = math.floor(rpm / 0.229) * 0.0239691227 * settings['wheel_radius']
    turn = 2 * math.floor(turn_rpm / 0.229) * 0.0239691227 * settings['wheel_radius'] / settings['wheel_separation']
    parameters.update({'linear.x.max_velocity': linear, 'linear.x.min_velocity': -linear,
                       'angular.z.max_velocity': turn, 'angular.z.min_velocity': -turn,
                       'linear.x.max_acceleration': 0.05, 'linear.x.max_deceleration': -0.05,
                       'angular.z.max_acceleration': 0.10*response_scale,
                       'angular.z.max_deceleration': -0.10*response_scale})
    # Independent axis limits form a box; bounded_twist in the follower scales
    # simultaneous translation/rotation to keep each commanded wheel <= rpm.
    validate_controller_limits(settings, parameters,
                               max_wheel_raw=math.floor(rpm/0.229)+math.floor(turn_rpm/0.229))


def build_geometry(settings):
    validate_settings(settings)
    description_path = Path(get_package_share_directory('orinbot_description')) / 'urdf/orinbot_hardware.urdf.xacro'
    mappings = {key: str(settings[key]) for key in (
        'profile_size', 'body_length', 'body_width', 'body_height', 'ground_clearance',
        'drive_axle_x', 'wheel_radius', 'wheel_separation', 'caster_x', 'caster_separation',
        'caster_radius', 'caster_width', 'caster_height', 'caster_plate_length', 'caster_plate_width')}
    mappings['wheel_width'] = str(settings.get('wheel_width') or 0)
    mappings['caster_trail'] = str(settings.get('caster_trail') if settings.get('caster_trail') is not None else 0.01)
    mappings['caster_trail_measured'] = str(settings.get('caster_trail') is not None).lower()
    robot = ET.fromstring(xacro.process_file(str(description_path), mappings=mappings).toxml())
    # The RealSense driver owns the calibrated camera_link -> optical/IMU TF tree.
    for sensor, child, rpy in (
            ('lidar', 'laser', (0, 0, math.radians(settings['lidar_yaw_degrees']))),
            ('camera', 'camera_link', (0, math.radians(settings['camera_pitch_degrees']), 0))):
        ET.SubElement(robot, 'link', name=child)
        joint = ET.SubElement(robot, 'joint', name=f'{sensor}_mount_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='base_footprint')
        ET.SubElement(joint, 'child', link=child)
        # User measurements are body-centered in x/y and floor-referenced in z.
        xyz = (settings[f'{sensor}_x'] - settings['drive_axle_x'],
               settings[f'{sensor}_y'], settings[f'{sensor}_z'])
        ET.SubElement(joint, 'origin',
                      xyz=' '.join(str(value) for value in xyz),
                      rpy=' '.join(str(value) for value in rpy))
    return robot


def build_description(settings, mock=False, torque=False):
    robot = build_geometry(settings)

    control = ET.SubElement(robot, 'ros2_control', name='orinbot_xm430', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = ('mock_components/GenericSystem' if mock
                                             else 'dynamixel_hardware_interface/DynamixelHardware')
    if mock:
        # Integrate velocity into encoder position for position-feedback odometry.
        ET.SubElement(hardware, 'param', name='calculate_dynamics').text = 'true'
    if not mock:
        left, right = settings['left_direction'], settings['right_direction']
        parameters = {
            'port_name': settings['motor_port'], 'baud_rate': settings['baud_rate'],
            'number_of_joints': 2, 'number_of_transmissions': 2,
            'transmission_to_joint_matrix': f'{left},0,0,{right}',
            'joint_to_transmission_matrix': f'{left},0,0,{right}',
            'dynamixel_model_folder': '/param/dxl_model', 'error_timeout_ms': 200,
            'disable_torque_at_init': 'false',
        }
        for key, value in parameters.items():
            ET.SubElement(hardware, 'param', name=key).text = str(value)
    for side in ('left', 'right'):
        joint = ET.SubElement(control, 'joint', name=f'{side}_wheel_joint')
        ET.SubElement(joint, 'command_interface', name='velocity')
        for interface in ('position', 'velocity'):
            ET.SubElement(joint, 'state_interface', name=interface)
        if not mock:
            gpio = ET.SubElement(control, 'gpio', name=f'{side}_motor')
            for key, value in {'type': 'dxl', 'ID': settings[f'{side}_id'],
                               'Operating Mode': 1, 'Torque Enable': int(torque),
                               'Goal Velocity': 0, 'Profile Acceleration': 5,
                               'Bus Watchdog': 15}.items():
                ET.SubElement(gpio, 'param', name=key).text = str(value)
            ET.SubElement(gpio, 'command_interface', name='Goal Velocity')
            # Read torque/goal registers themselves: driver 1.5.2's dxl_state
            # torque_state cache is not updated. These appear in dynamic_joint_states.
            for interface in ('Present Position', 'Present Velocity', 'Hardware Error Status',
                              'Torque Enable', 'Goal Velocity'):
                ET.SubElement(gpio, 'state_interface', name=interface)
    return ET.tostring(robot, encoding='unicode')


def setup(context):
    share = Path(get_package_share_directory('orinbot_hardware'))
    with open(LaunchConfiguration('config').perform(context)) as source:
        settings = yaml.safe_load(source)
    validate_settings(settings)
    mock = LaunchConfiguration('mock_hardware').perform(context) == 'true'
    torque = LaunchConfiguration('enable_torque').perform(context) == 'true'
    with open(share / 'config/controllers.yaml') as source:
        controllers = yaml.safe_load(source)
    straight_rpm = float(LaunchConfiguration('straight_test_rpm').perform(context))
    rectangle_rpm = float(LaunchConfiguration('rectangle_rpm').perform(context))
    parameters = controllers['diff_drive_controller']['ros__parameters']
    if straight_rpm and rectangle_rpm:
        raise ValueError('Select only one speed profile')
    if rectangle_rpm:
        configure_rectangle_profile(settings, parameters, rectangle_rpm,
            float(LaunchConfiguration('rectangle_turn_rpm').perform(context)))
    elif straight_rpm:
        configure_straight_speed_test(settings, parameters, straight_rpm)
    else:
        validate_controller_limits(settings, parameters)
    if not mock:
        verify_bus(settings)
    description = build_description(settings, mock, torque)
    controllers['diff_drive_controller']['ros__parameters'].update({
        key: float(settings[key]) for key in ('wheel_radius', 'wheel_separation')})
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='orinbot_controllers_', delete=False) as target:
        yaml.safe_dump(controllers, target)
        controller_file = target.name

    def cleanup(_context, **_kwargs):
        Path(controller_file).unlink(missing_ok=True)
        return []

    manager = Node(package='controller_manager', executable='ros2_control_node', output='screen',
                   parameters=[controller_file, {'use_sim_time': False}])
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[
            EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description, 'use_sim_time': False}]),
        manager,
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', '-c', '/controller_manager', '-p', controller_file]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['diff_drive_controller', '-c', '/controller_manager', '-p', controller_file,
                        '--controller-ros-args', '--remap ~/cmd_vel:=/cmd_vel --remap ~/odom:=/odom']),
    ]


def generate_launch_description():
    default_config = str(Path(get_package_share_directory('orinbot_hardware')) / 'config/hardware.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('mock_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_torque', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('straight_test_rpm', default_value='0',
                              description='Explicit straight-only speed test, 1-60 rpm; 0 uses normal limits'),
        DeclareLaunchArgument('rectangle_rpm', default_value='0',
                              description='Rectangle profile: 1-60 rpm straight'),
        DeclareLaunchArgument('rectangle_turn_rpm', default_value='10',
                              description='Rectangle turn wheel speed: 1-25 rpm, capped by rectangle_rpm'),
        OpaqueFunction(function=setup),
    ])
