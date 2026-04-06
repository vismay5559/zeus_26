import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your new XML file
    zeus_bringup_dir = get_package_share_directory('zeus_bringup')
    fastdds_profile_path = os.path.join(zeus_bringup_dir, 'config', 'fastdds_shm.xml')

    # 1. THE LOCK: Force all nodes launched by this file to use SHM
    shm_env_var = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE',
        value=fastdds_profile_path
    )

    # 2. Launch the C++ Hardware Interface
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # ... your controller params ...
    )

    # 3. Launch the Python RL Policy
    rl_policy_node = Node(
        package='zeus_control_interface',
        executable='rl_policy_node',
    )

    return LaunchDescription([
        shm_env_var,
        controller_manager_node,
        rl_policy_node
    ])