import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue # Make sure this is imported

def generate_launch_description():
    pkg_name = 'krsri2025'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    robot_description_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    trajectory_params_path = os.path.join(pkg_share, 'config', 'trajectory_params.yaml')

    # Get the URDF content from the xacro file
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_description_path
    ])

    return LaunchDescription([
        LogInfo(msg=['Using robot description from: ', robot_description_path]),

        Node(
            package='krsri2025',
            executable='send_trajectory',
            name='send_trajectory',
            output='screen',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description_content,
                        value_type=str
                    )
                },
                trajectory_params_path
            ]
        )
    ])
