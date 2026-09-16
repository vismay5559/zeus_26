"""
Show the model, no robot needed.

    ros2 launch zeus_description display.launch.py            # TF only
    ros2 launch zeus_description display.launch.py gui:=true  # sliders + RViz

gui:=true needs joint_state_publisher_gui and rviz2, which come with
ros-jazzy-desktop but not ros-jazzy-ros-base. Install them on a laptop:
    sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2

On the robot, do not use this - zeus_bringup's robot.launch.py runs
robot_state_publisher against the real /joint_states from the STM32.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gui = LaunchConfiguration('gui')

    # Generated from the CAD export by zeus_description/scripts/clean_urdf.py.
    urdf = os.path.join(get_package_share_directory('zeus_description'), 'urdf', 'zeus.urdf')
    with open(urdf) as f:
        robot_description = ParameterValue(f.read(), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='joint sliders and RViz'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui), output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(gui), output='screen'),
    ])
