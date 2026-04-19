"""
Gazebo Fortress + MoveIt2 simulation launch for Madara 6-DoF arm.
Single command — no need to run moveit_rviz.launch.py separately.
use_sim_time=True is applied to every node automatically.

Usage:
  ros2 launch madara_bringup gz_launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    desc_pkg  = FindPackageShare('madara_description').find('madara_description')
    bring_pkg = FindPackageShare('madara_bringup').find('madara_bringup')

    urdf_path = os.path.join(desc_pkg, 'urdf', 'madara_gz.urdf.xacro')
    ctrl_yaml = os.path.join(bring_pkg, 'config', 'ros2_controllers.yaml')

    # ── Launch arguments ────────────────────────────────────────────────────
    world = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file (default: built-in empty.sdf)')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str)

    # ── Robot state publisher — sim time ────────────────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }])

    # ── Ignition Gazebo ─────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true',
        }.items())

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'madara',
            '-x', '0', '-y', '0', '-z', '0.01',
        ],
        output='screen')

    # ── ROS-Gazebo bridges ──────────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen')

    gz_image_bridge = Node(
        package='ros_gz_image', executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen')

    # ── Controller spawners — gated on robot spawn ──────────────────────────
    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '-c', '/controller_manager',
                   '--ros-args', '-p', 'use_sim_time:=true'])

    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['madara_arm_controller',
                   '-c', '/controller_manager',
                   '--ros-args', '-p', 'use_sim_time:=true'])

    spawn_hand = Node(
        package='controller_manager', executable='spawner',
        arguments=['hand_controller',
                   '-c', '/controller_manager',
                   '--ros-args', '-p', 'use_sim_time:=true'])

    after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[spawn_jsb]))
    after_jsb   = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb,   on_exit=[spawn_arm]))
    after_arm   = RegisterEventHandler(
        OnProcessExit(target_action=spawn_arm,   on_exit=[spawn_hand]))

    # ── MoveIt2 — launched after controllers are up (5 s delay) ────────────
    # use_sim_time=True is passed so move_group reads /clock from Gazebo,
    # matching the timestamps on joint_states from joint_state_broadcaster.
    moveit_config = (
        MoveItConfigsBuilder("madara", package_name="madara_moveit_config")
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'madara.urdf.xacro'))
        .robot_description_semantic(file_path="srdf/madara.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},          # ← must match Gazebo clock
        ],
        arguments=["--ros-args", "--log-level", "info"])

    rviz_cfg = os.path.join(
        get_package_share_directory("madara_moveit_config"), "rviz", "moveit.rviz")

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},          # ← must match Gazebo clock
        ])

    # Delay MoveIt by 5 s to give Gazebo + controllers time to come up
    moveit_delayed = TimerAction(period=5.0, actions=[move_group, rviz])

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(rsp_node)
    ld.add_action(gz_sim)
    ld.add_action(gz_bridge)
    ld.add_action(gz_image_bridge)
    ld.add_action(spawn_robot)
    ld.add_action(after_robot)
    ld.add_action(after_jsb)
    ld.add_action(after_arm)
    ld.add_action(moveit_delayed)
    return ld