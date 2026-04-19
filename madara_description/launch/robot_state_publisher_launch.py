import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time   = LaunchConfiguration('use_sim_time')
    urdf_model     = LaunchConfiguration('urdf_model')

    pkg_path = FindPackageShare('madara_description').find('madara_description')
    default_urdf  = os.path.join(pkg_path, 'urdf', 'madara.urdf.xacro')

    robot_description = Command(['xacro ', urdf_model])

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_urdf = DeclareLaunchArgument(
        'urdf_model', default_value=default_urdf,
        description='Full path to the URDF/xacro file')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }])

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(declare_urdf)
    ld.add_action(rsp_node)
    return ld