import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('zeus_sensor_fusion')

    kinematics_config = os.path.join(pkg_dir, 'config', 'kinematics.yaml')
    inekf_config = os.path.join(pkg_dir, 'config', 'inekf_params.yaml')

    sensor_fusion_node = Node(
        package='zeus_sensor_fusion',
        executable='sensor_fusion_node',
        name='zeus_sensor_fusion',
        parameters=[kinematics_config, inekf_config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        sensor_fusion_node,
    ])
