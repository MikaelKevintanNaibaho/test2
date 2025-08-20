# In description/launch/send_trajectory.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("krsri2025"),
                    "urdf",
                    "robot.xacro",
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- CORRECTED CODE ---
    # Load your existing trajectory parameters file
    trajectory_params_config = os.path.join(
        get_package_share_directory('krsri2025'),
        'config',
        'trajectory_params.yaml' # Use your existing file
    )
    # --- END CORRECTION ---

    send_trajectory_node = Node(
        package="krsri2025",
        executable="send_trajectory",
        output="screen",
        # Load the robot description and your trajectory params
        parameters=[robot_description, trajectory_params_config],
    )

    return LaunchDescription(
        [
            send_trajectory_node,
        ]
    )
