from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    # Get package directory
    pkg_name = 'krsri2025'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths to config files
    robot_description_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    controllers_path = os.path.join(pkg_share, 'config', 'controller.yaml')
    joint_pose_path = os.path.join(pkg_share, 'config', 'joint_pose.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_description_path
    ])
    
    # Wrap the robot description as a string parameter
    robot_description = {'robot_description': ParameterValue(
        robot_description_content, 
        value_type=str
    )}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,
                    {'use_sim_time': False}
        ]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            joint_pose_path,
            controllers_path
            
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Robot Controller
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['krsri_controller'],
        output='screen',
    )
    

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Delay rviz start after Joint State Broadcaster
    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ])
