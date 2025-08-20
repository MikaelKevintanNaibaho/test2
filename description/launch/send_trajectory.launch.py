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

    send_trajectory_node = Node(
        package="krsri2025",
        executable="send_trajectory",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            send_trajectory_node,
        ]
    )
