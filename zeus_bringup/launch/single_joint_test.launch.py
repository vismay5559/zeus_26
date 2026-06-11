import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = os.path.join(
        get_package_share_directory('zeus_bringup'),
        'config',
        'single_joint_controllers.yaml',
    )

    # In Jazzy, ros2_control_node subscribes to /robot_description topic instead
    # of reading a parameter. robot_state_publisher is what publishes that topic.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('zeus_description'),
                    'urdf',
                    'single_joint_test_robot.xacro',
                ]),
            ]),
            value_type=str,
        )
    }

    # Publishes robot_description to /robot_description topic (required by Jazzy ros2_control)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    forward_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_command_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_spawner,
        forward_controller_spawner,
    ])
