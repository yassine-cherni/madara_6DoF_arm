import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_path        = FindPackageShare('madara_description').find('madara_description')
    rviz_config     = os.path.join(pkg_path, 'rviz', 'description.rviz')
    default_urdf    = os.path.join(pkg_path, 'urdf', 'madara.urdf.xacro')

    gui              = LaunchConfiguration('use_gui')
    urdf_model       = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    declare_urdf = DeclareLaunchArgument(
        'urdf_model', default_value=default_urdf,
        description='Absolute path to robot URDF/xacro file')

    declare_rviz = DeclareLaunchArgument(
        'rviz_config_file', default_value=rviz_config,
        description='Full path to the RViz config file')

    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_gui = DeclareLaunchArgument(
        'use_gui', default_value='False',
        description='Use joint_state_publisher_gui if true')

    jsp_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    jsp_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'robot_state_publisher_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf_model':   urdf_model,
        }.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    ld = LaunchDescription()
    ld.add_action(declare_urdf)
    ld.add_action(declare_rviz)
    ld.add_action(declare_gui)
    ld.add_action(declare_sim)
    ld.add_action(jsp_node)
    ld.add_action(jsp_gui_node)
    ld.add_action(rsp)
    ld.add_action(rviz_node)
    return ld