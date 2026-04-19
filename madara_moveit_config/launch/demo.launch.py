"""
MoveIt2 demo launch — uses mock_components (no real hardware needed).
Spawns: move_group, ros2_control_node (mock), controllers, RViz.

Usage:
  ros2 launch madara_moveit_config demo.launch.py

FIX: Controller spawners now use OnProcessExit event handlers so they
     fire sequentially:  joint_state_broadcaster → madara_arm_controller
     → hand_controller.  The original launch fired all three at once,
     which races against the controller_manager not being ready yet.
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("madara", package_name="madara_moveit_config")
        .robot_description(file_path="urdf/fake_madara.urdf.xacro")
        .robot_description_semantic(file_path="srdf/madara.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ros2_controllers_yaml = os.path.join(
        get_package_share_directory("madara_bringup"),
        "config", "ros2_controllers.yaml")

    # ── Core nodes ──────────────────────────────────────────────────────────
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description])

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_yaml],
        output="screen")

    # ── Controller spawners — sequential via OnProcessExit ──────────────────
    # FIX: previously all three spawners were placed directly in
    # LaunchDescription and fired simultaneously.  This races because
    # the arm and hand spawners need joint_state_broadcaster to be
    # fully active first.  Chain: jsb done → arm starts → arm done → hand.
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"])

    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["madara_arm_controller", "-c", "/controller_manager"])

    hand_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"])

    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner]))

    after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[hand_spawner]))

    # ── RViz ────────────────────────────────────────────────────────────────
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
        ])

    return LaunchDescription([
        move_group,
        rsp,
        static_tf,
        ros2_control_node,
        jsb_spawner,   # fires first; arm_spawner triggered by after_jsb
        after_jsb,     # jsb done  → arm_spawner
        after_arm,     # arm done  → hand_spawner
        rviz,
    ])
