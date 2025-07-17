# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Lancement minimal pour extrusion uniquement 
# Ne lance que : 
# - la description du robot robot_description
# - ros2_control_node
# - joint_state_broadcaster
# - gpio_command_controller 

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo

def generate_launch_description():
     # Description du robot (nécessaire à ros2_control)
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
     
     log_message = LogInfo(msg=(
        "[MODE EXTRUSION UNIQUEMENT]"
        "Merci d'ouvrir un autre terminal pour publier sur le topic "
        " `gpio_command_controller/commands` avec les vitesses de rotation  "
        " de la viscotec souhaitée (en step/s)"
     ))

     return LaunchDescription([
         log_message,
         control_node,
         joint_state_broadcaster_spawner,
         gpio_command_controller_spawner
     ])

