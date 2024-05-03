from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('printer3d_keyence_data_acquisition_test'),
        'config',
        'params.yaml'
        )
        
    with open(config, 'r') as f:
        configYaml = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='printer3d_keyence_profile_capture',
            namespace='measure_process',
            executable='keyence_control_node.py',
            name='keyence_control_node'
        ),
        Node(
            package='printer3d_keyence_data_acquisition_test',
            namespace='measure_process',
            executable='keyence_commands_node.py',
            name='keyence_commands_node'
        )
    ])