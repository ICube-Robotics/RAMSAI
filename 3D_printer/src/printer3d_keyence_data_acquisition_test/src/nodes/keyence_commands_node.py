#!/usr/bin/env python3
# Copyright 2023 ICube Laboratory, University of Strasbourg
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

import rclpy
from rclpy.node import Node
import time
import ctypes
import sys
import numpy as np
from printer3d_keyence_msgs.srv import KeyenceControlCommands, SaveProfilesCommand

class KeyenceCommandNode(Node):
    def __init__(self):
        """KeyenceCommandNode class constructor."""
        super().__init__('printer3d_keyence_data_acquisition_test')

        self.client_keyence_control_service = self.create_client(KeyenceControlCommands, 'keyence_control_commands')
        while not self.client_keyence_control_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('keyence control service not available, waiting again...')
        self.req_keyence_control = KeyenceControlCommands.Request()

        self.client_saving_buffer_service = self.create_client(SaveProfilesCommand, 'saving_buffer')
        while not self.client_saving_buffer_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('buffer saving service not available, waiting again...')
        self.req_saving_buffer = SaveProfilesCommand.Request()

    def send_command(self,command):
        self.req_keyence_control.order = command
        self.future_keyence_control = self.client_keyence_control_service.call_async(self.req_keyence_control)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_keyence_control.done():
                self.get_logger().info('command executed')
                return 0

    def ask_for_buffer_saving(self,filename):
        self.req_saving_buffer.filenametosave = filename
        self.future_buffer_saving = self.client_saving_buffer_service.call_async(self.req_saving_buffer)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_buffer_saving.done():
                self.get_logger().info('command executed')
                return 0

    def sendImageCaptureRequest(self, filename):
        self.req_image_capture.filename = filename
        self.future_image_capture = self.client_image_capture.call_async(self.req_image_capture)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future_image_capture.done():
                self.get_logger().info('capture finished')
                break

if __name__ == '__main__':
    rclpy.init()

    keyence_command_node = KeyenceCommandNode()
    keyence_command_node.send_command('laser_on')
    keyence_command_node.send_command('buffer_flush')
    keyence_command_node.get_logger().info("10 secondes pour acquerir des profils")
    time.sleep(10)
    keyence_command_node.get_logger().info("lancement sauvegarde")
    keyence_command_node.send_command('laser_off')
    keyence_command_node.ask_for_buffer_saving('/home/gulltor/Desktop/Keyence_control/LJ-X8000A_PyLib_1_3_0_2/ros2_keyence_controler/profiles.npy')
    keyence_command_node.send_command('laser_on')




    rclpy.shutdown()