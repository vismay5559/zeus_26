"""
The robot plus something that commands it.

    ros2 launch zeus_bringup walk.launch.py                          # watch: stays IDLE
    ros2 launch zeus_bringup walk.launch.py enable:=true             # walk the stored gait
    ros2 launch zeus_bringup walk.launch.py policy:=rl enable:=true  # your RL policy

    policy   passthrough   zero residual: the STM32's stored gait, uncorrected
             rl            zeus_control_interface rl_policy_node
    enable   false keeps the board IDLE while commands still flow (safe default)

Every robot.launch.py argument (port, rerun, rerun_host, ...) works here too.

To stop moving without killing anything:
    ros2 service call /zeus/stand_down std_srvs/srv/Trigger
    ros2 service call /zeus/resume     std_srvs/srv/Trigger
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    policy = LaunchConfiguration('policy')
    enable = ParameterValue(LaunchConfiguration('enable'), value_type=bool)

    def is_policy(name):
        return IfCondition(PythonExpression(["'", policy, "' == '", name, "'"]))

    return LaunchDescription([
        DeclareLaunchArgument('policy', default_value='passthrough',
                              choices=['passthrough', 'rl']),
        DeclareLaunchArgument('enable', default_value='false'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('zeus_bringup'), 'launch', 'robot.launch.py']))),

        Node(package='zeus_link', executable='gait_passthrough_node', name='gait_passthrough',
             condition=is_policy('passthrough'), parameters=[{'enable': enable}],
             output='screen', emulate_tty=True),

        Node(package='zeus_control_interface', executable='rl_policy_node',
             name='rl_policy', condition=is_policy('rl'), parameters=[{'enable': enable}],
             output='screen', emulate_tty=True),
    ])
