"""
Real camera launch — Raspberry Pi Camera Module v2 on Raspberry Pi 5.
Uses camera_ros (libcamera backend) — the correct driver for RPi 5.

Run alongside bringup_launch.py:
  Terminal 1: ros2 launch madara_bringup bringup_launch.py
  Terminal 2: ros2 launch madara_bringup camera_launch.py

Published topics:
  /camera/image_raw      (sensor_msgs/msg/Image)
  /camera/camera_info    (sensor_msgs/msg/CameraInfo)

Requirements (inside Docker container or on Pi):
  sudo apt install ros-humble-camera-ros

If camera_ros is not available, fallback to v4l2_camera:
  sudo apt install ros-humble-v4l2-camera
  Change the node below to use package='v4l2_camera', executable='v4l2_camera_node'
  and remove the camera:=0 param — it uses video_device:='/dev/video0' instead.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────────
    declare_width = DeclareLaunchArgument(
        'width', default_value='1280',
        description='Camera capture width in pixels')

    declare_height = DeclareLaunchArgument(
        'height', default_value='720',
        description='Camera capture height in pixels')

    declare_fps = DeclareLaunchArgument(
        'fps', default_value='30',
        description='Camera frame rate (frames per second)')

    width  = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps    = LaunchConfiguration('fps')

    # ── camera_ros node (libcamera backend — correct for RPi 5) ────────────
    # Publishes: /camera/image_raw, /camera/camera_info
    # Remaps to standard topic names used by the rest of the stack.
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'camera':  0,        # Camera index — 0 = first camera (RPi Camera v2)
            'width':   width,
            'height':  height,
            'fps':     fps,
            # Frame ID must match the optical TF frame in the URDF
            'frame_id': 'camera_link_optical',
        }],
        remappings=[
            # camera_ros publishes on ~/image_raw and ~/camera_info
            # Remap to the flat /camera/... namespace used by the arm stack
            ('~/image_raw',   '/camera/image_raw'),
            ('~/camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    # ── static camera info publisher (until you run full calibration) ───────
    # This publishes a basic CameraInfo with the real RPi v2 specs so
    # OpenCV-based nodes (ArUco, etc.) have something to work with.
    # After calibrating with camera_calibration, replace this with the
    # calibration YAML and use camera_info_manager instead.
    camera_info_node = Node(
        package='camera_info_manager',
        executable='camera_info_manager_node',
        name='camera_info_manager',
        namespace='camera',
        parameters=[{
            'camera_name': 'rpi_v2',
            # Intrinsics: RPi Camera v2 at 1280×720, approximate factory values.
            # Calibrate yours with: ros2 run camera_calibration cameracalibrator
            # and replace these with your calibration file URL.
            'camera_info_url': '',  # leave empty to use defaults below
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_width,
        declare_height,
        declare_fps,
        camera_node,
        # camera_info_node,  # uncomment once you have a calibration file
    ])