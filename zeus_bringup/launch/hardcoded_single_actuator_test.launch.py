import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    node_id = LaunchConfiguration('node_id')
    target = LaunchConfiguration('target')

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

    hardcoded_test_node = Node(
        package='zeus_control_interface',
        executable='hardcoded_actuator_test_node',
        parameters=[{
            'target': target,
            'start_delay_sec': 4.0,
            'hold_zero_sec': 2.0,
            'hold_target_sec': 2.0,
            'publish_rate_hz': 20.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('node_id', default_value='1'),
        DeclareLaunchArgument('target', default_value='0.02'),
        controller_manager,
        joint_state_spawner,
        command_controller_spawner,
        TimerAction(period=4.0, actions=[hardcoded_test_node]),
    ])
