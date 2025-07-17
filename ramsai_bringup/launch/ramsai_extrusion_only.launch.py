from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     # Get URDF
     robot_description_content = Command(
         [
             PathJoinSubstitution([FindExecutable(name='xacro')]),
             ' ',
             PathJoinSubstitution([
                 FindPackageShare('ramsai_description'), 'config',
'ramsai.config.xacro'
             ]),
             ' ',
             'prefix:="" ',
             'use_sim:=false ',
             'use_fake_hardware:=true ',
             'robot_ip:=192.170.10.3 ',
             'robot_port:=30200 ',
             'initial_positions_file:=initial_positions.yaml ',
             'command_interface:=position ',
             'base_frame_file:=base_frame.yaml'
         ]
     )
     robot_description = {'robot_description': robot_description_content}

     # Load controllers config
     robot_controllers = PathJoinSubstitution([
         FindPackageShare('ramsai_description'),
         'config',
         'ramsai_controllers.yaml',
     ])

     # Control node
     control_node = Node(
         package='controller_manager',
         executable='ros2_control_node',
         parameters=[robot_description, robot_controllers],
         output='both'
     )

     # Spawner for joint_state_broadcaster
     joint_state_broadcaster_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['joint_state_broadcaster', '--controller-manager',
'/controller_manager'],
     )

     # Spawner for GPIO controller
     gpio_command_controller_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['gpio_command_controller', '--controller-manager',
'/controller_manager'],
     )

     return LaunchDescription([
         control_node,
         joint_state_broadcaster_spawner,
         gpio_command_controller_spawner
     ])

