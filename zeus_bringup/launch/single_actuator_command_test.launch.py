import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    node_id = LaunchConfiguration('node_id')

    controllers_file = os.path.join(
        get_package_share_directory('zeus_bringup'),
        'config',
        'single_actuator_command_test.yaml',
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('zeus_description'),
                'urdf',
                'single_actuator_command_test.urdf.xacro',
            ]),
            ' can_interface:=',
            can_interface,
            ' node_id:=',
            node_id,
        ])
    }

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    command_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['single_actuator_command_controller'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('node_id', default_value='1'),
        controller_manager,
        joint_state_spawner,
        command_controller_spawner,
    ])
