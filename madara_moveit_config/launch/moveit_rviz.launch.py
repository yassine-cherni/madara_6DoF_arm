"""
MoveIt2 + RViz overlay for real hardware or Gazebo simulation.
Run this AFTER gz_launch.py (simulation) or bringup_launch.py (real HW).

Usage:
  ros2 launch madara_moveit_config moveit_rviz.launch.py                        # real HW
  ros2 launch madara_moveit_config moveit_rviz.launch.py use_sim_time:=True     # Gazebo
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch argument ────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if True')

    desc_pkg = FindPackageShare("madara_description").find("madara_description")
    urdf_path = os.path.join(desc_pkg, "urdf", "madara.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("madara", package_name="madara_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="srdf/madara.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── move_group node — use_sim_time now comes from launch arg ───────
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},   # ← was hardcoded False, now dynamic
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
            {"use_sim_time": use_sim_time},   # ← also pass to RViz
        ])

    return LaunchDescription([
        declare_use_sim_time,
        move_group,
        rviz,
    ])