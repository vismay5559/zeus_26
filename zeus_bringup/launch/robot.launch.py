"""
Bring up the Pi side of the robot: the STM32 link, the model, and optionally Rerun.

    ros2 launch zeus_bringup robot.launch.py
    ros2 launch zeus_bringup robot.launch.py rerun:=connect rerun_host:=192.168.1.50
    ros2 launch zeus_bringup robot.launch.py rerun:=save rerun_path:=/home/pi/run.rrd

Nothing here commands the robot. The STM32 stays in BOOT/IDLE until something
publishes /zeus/command - use walk.launch.py for that.

Arguments
    port          auto | /dev/ttyACMn            (auto finds the board by USB ID)
    rerun         off | connect | save
    rerun_host    the laptop running `rerun`, for connect
    rerun_path    .rrd file, for save
    rerun_decimate  log every Nth state (10 = 100 Hz)
    rerun_degrees   true = joint angles in degrees
    shm_only      true = Fast DDS shared memory only. Lower jitter on the Pi,
                  but no topic is visible from another machine.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = get_package_share_directory('zeus_bringup')

    port = LaunchConfiguration('port')
    rerun = LaunchConfiguration('rerun')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [FindPackageShare('zeus_description'), 'urdf', 'zeus.urdf.xacro'])]),
        value_type=str)

    rerun_on = IfCondition(PythonExpression(["'", rerun, "' != 'off'"]))

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('rerun', default_value='off',
                              choices=['off', 'connect', 'save']),
        DeclareLaunchArgument('rerun_host', default_value=''),
        DeclareLaunchArgument('rerun_path', default_value='zeus.rrd'),
        DeclareLaunchArgument('rerun_decimate', default_value='10'),
        DeclareLaunchArgument('rerun_degrees', default_value='false'),
        DeclareLaunchArgument('shm_only', default_value='false'),

        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup, 'config', 'fastdds_shm.xml'),
            condition=IfCondition(LaunchConfiguration('shm_only'))),

        Node(package='zeus_link', executable='link_node', name='zeus_link',
             parameters=[os.path.join(bringup, 'config', 'link.yaml'), {'port': port}],
             output='screen', emulate_tty=True),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),

        Node(package='zeus_rerun', executable='rerun_node', name='zeus_rerun',
             condition=rerun_on,
             parameters=[{
                 'mode': rerun,
                 'host': LaunchConfiguration('rerun_host'),
                 'path': LaunchConfiguration('rerun_path'),
                 'decimate': ParameterValue(LaunchConfiguration('rerun_decimate'),
                                            value_type=int),
                 'degrees': ParameterValue(LaunchConfiguration('rerun_degrees'),
                                           value_type=bool),
             }],
             output='screen', emulate_tty=True),
    ])
