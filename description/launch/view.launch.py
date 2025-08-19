from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and joint state publisher gui automatica;lly \ with this launch file.",
        )
    )

    # init arguments
    gui = LaunchConfiguration("gui")
    # ambil URDF dari Xacro file robot
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

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("krsri2025"), "rviz", "view_robot.rviz"]
    )

    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes_to_start = [
        joint_state_pub_node,
        robot_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
