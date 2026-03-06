import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = get_package_share_directory('bravo_cathode_protection')
    urdf_file_name = 'urdf/bravo_5_dynamics_pinocchio_cp.urdf'
    urdf_path = os.path.join(pkg_share, urdf_file_name)
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description_content = urdf_file.read()

    rviz_config_file = os.path.join(pkg_share, 'rviz/rviz.rviz')
    rviz_arguments = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Robot state publisher.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description_content
            }]),

        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='RVIZ',
            arguments=rviz_arguments
        )
    ])
