# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     # Load robot description from xacro
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

     # Load controllers config file
     robot_controllers = PathJoinSubstitution([
         FindPackageShare('ramsai_description'),
         'config',
         'ramsai_controllers.yaml',
     ])

     # Core ROS2 control node
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

     # Spawner for gpio_command_controller
     gpio_command_controller_spawner = Node(
         package='controller_manager',
         executable='spawner',
         arguments=['gpio_command_controller', '--controller-manager',
'/controller_manager'],
     )

     # Node for receiving velocity from SunRise (Viscotec)
     viscotec_node = Node(
         package='ramsai_nodes',
         executable='viscotec_velocity_observer',
         name='viscotec_velocity_observer',
         output='screen',
     )

     # Node for publishing VM fake (SunRise simulation)
     fake_vm_node = Node(
         package='ramsai_nodes',
         executable='fake_vm_publisher',
         name='fake_vm_publisher',
         output='log',
     )

     return LaunchDescription([
         control_node,
         joint_state_broadcaster_spawner,
         gpio_command_controller_spawner,
         viscotec_node,
         #fake_vm_node,
     ])
