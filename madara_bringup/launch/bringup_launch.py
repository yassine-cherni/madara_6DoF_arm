"""
Real-hardware bringup for Madara 6DoF arm.
Starts: ros2_control, controllers, and RPi Camera v2 (v4l2_camera).

Usage:
  ros2 launch madara_bringup bringup_launch.py
  ros2 launch madara_bringup bringup_launch.py serial_port:=/dev/ttyACM0
  ros2 launch madara_bringup bringup_launch.py start_camera:=false
  ros2 launch madara_bringup bringup_launch.py video_device:=/dev/video2
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue   # ← fix


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    desc_pkg  = FindPackageShare('madara_description').find('madara_description')
    bring_pkg = FindPackageShare('madara_bringup').find('madara_bringup')

    urdf_path = os.path.join(desc_pkg,  'urdf',   'real_madara.urdf.xacro')
    ctrl_yaml = os.path.join(bring_pkg, 'config', 'ros2_controllers.yaml')

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_serial = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='UART device for Arduino Uno (/dev/ttyACM0 for USB, or /dev/ttyUSB0)')

    declare_camera = DeclareLaunchArgument(
        'start_camera', default_value='true',
        description='Launch the RPi Camera v2 node (v4l2_camera)')

    declare_video = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='V4L2 device path for the camera')

    serial_port  = LaunchConfiguration('serial_port')
    start_camera = LaunchConfiguration('start_camera')
    video_device = LaunchConfiguration('video_device')

    # ── Robot description ────────────────────────────────────────────────────
    # ParameterValue(value_type=str) required on Humble — without it the
    # node parameter parser tries to interpret the xacro output as YAML
    # and fails with "Unable to parse the value of parameter robot_description"
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' serial_port:=', serial_port]),
        value_type=str
    )

    # ── Nodes ────────────────────────────────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, ctrl_yaml],
        output='screen',
        remappings=[('~/robot_description', '/robot_description')],
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'])

    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['madara_arm_controller', '-c', '/controller_manager'])

    spawn_hand = Node(
        package='controller_manager', executable='spawner',
        arguments=['hand_controller', '-c', '/controller_manager'])

    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_arm]))
    after_arm = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm, on_exit=[spawn_hand]))

    # ── Camera node (v4l2_camera) ─────────────────────────────────────────
    # Reads /dev/video0 (passed into Docker from Pi OS host via privileged mode).
    # Publishes: /camera/image_raw, /camera/camera_info
    # frame_id must match camera_link_optical TF frame in rpi_v2_camera.xacro
    camera_node = Node(
        condition=IfCondition(start_camera),
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        namespace='camera',
        parameters=[{
            'video_device':    video_device,
            'image_size':      [1280, 720],
            'camera_frame_id': 'camera_link_optical',
            'pixel_format':    'YUYV',   # default for RPi Camera v2 via V4L2
            'output_encoding': 'rgb8',   # what OpenCV/ArUco nodes expect
        }],
        remappings=[
            ('~/image_raw',   '/camera/image_raw'),
            ('~/camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_serial,
        declare_camera,
        declare_video,
        rsp_node,
        control_node,
        spawn_jsb,
        after_jsb,
        after_arm,
        camera_node,
    ])
