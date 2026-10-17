import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('pick_and_place_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'arm_sim.yaml')

    arm_sim_node = Node(
        package='pick_and_place_control',
        executable='arm_sim_node',
        name='arm_sim_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        arm_sim_node
    ])