from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('keyence_controler'),
        'config',
        'params.yaml'
        )
        
    with open(config, 'r') as f:
        configYaml = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='keyence_controler',
            namespace='measure_process',
            executable='keyence_control_node.py',
            name='keyence_control_node'
        )
    ])