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

import LJXAwrap
import rclpy
from rclpy.node import Node
import time
import ctypes
import sys
import numpy as np
from printer3d_keyence_msgs.srv import KeyenceControlCommands,SaveProfilesCommand

class KeyenceControlNode(Node):
    def __init__(self):
        """KeyenceControlNode class constructor."""
        super().__init__('printer3d_keyence_profile_capture')

        self.declare_parameter('deviceId', 0)
        self.declare_parameter('abyIpAddress_1', 192)
        self.declare_parameter('abyIpAddress_2', 168)
        self.declare_parameter('abyIpAddress_3', 0)
        self.declare_parameter('abyIpAddress_4', 1)
        self.declare_parameter('wPortNo', 24691)
        self.declare_parameter('HighSpeedPortNo', 24692)

        self.deviceId = self.get_parameter('deviceId').get_parameter_value().integer_value
        self.ethernetConfig = LJXAwrap.LJX8IF_ETHERNET_CONFIG()
        self.ethernetConfig.abyIpAddress[0] = self.get_parameter('abyIpAddress_1').get_parameter_value().integer_value
        self.ethernetConfig.abyIpAddress[1] = self.get_parameter('abyIpAddress_2').get_parameter_value().integer_value
        self.ethernetConfig.abyIpAddress[2] = self.get_parameter('abyIpAddress_3').get_parameter_value().integer_value
        self.ethernetConfig.abyIpAddress[3] = self.get_parameter('abyIpAddress_4').get_parameter_value().integer_value
        self.ethernetConfig.wPortNo = self.get_parameter('wPortNo').get_parameter_value().integer_value
        self.HighSpeedPortNo = self.get_parameter('HighSpeedPortNo').get_parameter_value().integer_value

        self.res = LJXAwrap.LJX8IF_EthernetOpen(self.deviceId, self.ethernetConfig)
        self.get_logger().info("LJXAwrap.LJX8IF_EthernetOpen:" + str(hex(self.res)))
        if self.res != 0:
            self.get_logger().info('Failed to connect to the Keyence sensor')
            sys.exit()

        # LJ-X 8080
        headmodel = ctypes.create_string_buffer(32)
        self.res = LJXAwrap.LJX8IF_GetHeadModel(self.deviceId, headmodel)
        self.get_logger().info('LJXAwrap.LJX8IF_GetHeadModel:'+str(hex(self.res))+'<headmodel>='+str(headmodel.value))

        # LJ-X 8080 serial number
        controllerSerial = ctypes.create_string_buffer(16)
        headSerial = ctypes.create_string_buffer(16)
        res = LJXAwrap.LJX8IF_GetSerialNumber(self.deviceId,
                                                controllerSerial, headSerial)
        self.get_logger().info('LJXAwrap.LJX8IF_GetSerialNumber:'+str(hex(self.res))+'<controllerSerial>='+str(controllerSerial.value)+'<headSerial>='+str(headSerial.value))

        LJXAwrap.LJX8IF_ClearMemory(self.deviceId)

        self.keyenceControlService = self.create_service(KeyenceControlCommands, 'keyence_control_commands', self.control_service_routine)
        self.savingBufferService = self.create_service(SaveProfilesCommand, 'saving_buffer', self.profile_saving_service_routine)

    def select_job(self,job_number):
        programNo_get = ctypes.c_ubyte()
        self.res = LJXAwrap.LJX8IF_GetActiveProgram(self.deviceId, programNo_get)
        if programNo_get.value != job_number:
            programNo_set = job_number
            res = LJXAwrap.LJX8IF_ChangeActiveProgram(self.deviceId, programNo_set)
        return 0

    def enable_laser(self):
        LJXAwrap.LJX8IF_ControlLaser(self.deviceId, 1)
        return 0

    def disable_laser(self):
        LJXAwrap.LJX8IF_ControlLaser(self.deviceId, 0)
        return 0

    def get_attention_status(self):
        attentionStatus = ctypes.c_ushort()
        self.res = LJXAwrap.LJX8IF_GetAttentionStatus(self.deviceId, attentionStatus)
        self.get_logger().info("LJXAwrap.LJX8IF_GetAttentionStatus:"+ str(hex(self.res))+"<AttentionStatus>="+ str(bin(attentionStatus.value)))
        return 0

    def ask_for_a_profile(self):

        self.get_logger().info("LJXAwrap.LJX8IF_Trigger : " + str(hex(LJXAwrap.LJX8IF_Trigger(self.deviceId))))

        # Change according to your controller settings.
        xpointNum = 3200            # Number of X points per one profile.
        withLumi = 1                # 1: luminance data exists, 0: not exists.

        # Specifies the position, etc. of the profiles to get.
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.req.byTargetBank = 0x0      # 0: active bank
        self.req.byPositionMode = 0x0    # 0: from current position
        self.req.dwGetProfileNo = 0x0    # use when position mode is "POSITION_SPEC"
        self.req.byGetProfileCount = 1   # the number of profiles to read.
        self.req.byErase = 0             # 0: Do not erase

        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()

        self.profinfo = LJXAwrap.LJX8IF_PROFILE_INFO()

        # Calculate the buffer size to store the received profile data.
        self.dataSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        self.dataSize += ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        self.dataSize += ctypes.sizeof(ctypes.c_uint) * xpointNum * (1 + withLumi)
        self.dataSize *= self.req.byGetProfileCount

        dataNumIn4byte = int(self.dataSize / ctypes.sizeof(ctypes.c_uint))
        self.profdata = (ctypes.c_int * dataNumIn4byte)()

        # Send command.
        self.res = LJXAwrap.LJX8IF_GetProfile(self.deviceId,
                                            self.req,
                                            self.rsp,
                                            self.profinfo,
                                            self.profdata,
                                            self.dataSize)

        self.get_logger().info("LJXAwrap.LJX8IF_GetProfile:" + str(hex(self.res)))

        if self.res != 0:
            self.get_logger().info("Failed to get profile.")
            sys.exit()

        headerSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        addressOffset_height = int(headerSize / ctypes.sizeof(ctypes.c_uint))
        addressOffset_lumi = addressOffset_height + self.profinfo.wProfileDataCount

        profile_data = [[None, None, None] for i in range(self.profinfo.wProfileDataCount)]

        for i in range(self.profinfo.wProfileDataCount):
            # Conver X data to the actual length in millimeters
            x_val_mm = (self.profinfo.lXStart + self.profinfo.lXPitch * i) / 100.0  # um
            x_val_mm /= 1000.0  # mm

            # Conver Z data to the actual length in millimeters
            z_val = self.profdata[addressOffset_height + i]

            if z_val <= -2147483645:  # invalid value
                z_val_mm = None
            else:
                z_val_mm = z_val / 100.0  # um
                z_val_mm /= 1000.0  # mm

            # Luminance data
            lumi_val = self.profdata[addressOffset_lumi + i]

            profile_data[i][0] = x_val_mm
            profile_data[i][1] = z_val_mm
            profile_data[i][2] = lumi_val
        
        profile_data = np.array(profile_data)

        return profile_data

    def get_profile_number(self, number):
        # Change according to your controller settings.
        xpointNum = 3200            # Number of X points per one profile.
        withLumi = 1                # 1: luminance data exists, 0: not exists.

        # Specifies the position, etc. of the profiles to get.
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.req.byTargetBank = 0x0      # 0: active bank
        self.req.byPositionMode = 0x2    # 2: specify position
        self.req.dwGetProfileNo = number    # use when position mode is "POSITION_SPEC"
        self.req.byGetProfileCount = 1   # the number of profiles to read.
        self.req.byErase = 0             # 0: Do not erase

        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()

        self.profinfo = LJXAwrap.LJX8IF_PROFILE_INFO()

        # Calculate the buffer size to store the received profile data.
        self.dataSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        self.dataSize += ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        self.dataSize += ctypes.sizeof(ctypes.c_uint) * xpointNum * (1 + withLumi)
        self.dataSize *= self.req.byGetProfileCount

        dataNumIn4byte = int(self.dataSize / ctypes.sizeof(ctypes.c_uint))
        self.profdata = (ctypes.c_int * dataNumIn4byte)()

        # Send command.
        self.res = LJXAwrap.LJX8IF_GetProfile(self.deviceId,
                                            self.req,
                                            self.rsp,
                                            self.profinfo,
                                            self.profdata,
                                            self.dataSize)
        if self.res != 0:
            self.get_logger().info("Failed to get profile.")
            sys.exit()

        headerSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        addressOffset_height = int(headerSize / ctypes.sizeof(ctypes.c_uint))
        addressOffset_lumi = addressOffset_height + self.profinfo.wProfileDataCount

        profile_data = [[None, None, None] for i in range(self.profinfo.wProfileDataCount)]

        for i in range(self.profinfo.wProfileDataCount):
            # Conver X data to the actual length in millimeters
            x_val_mm = (self.profinfo.lXStart + self.profinfo.lXPitch * i) / 100.0  # um
            x_val_mm /= 1000.0  # mm

            # Conver Z data to the actual length in millimeters
            z_val = self.profdata[addressOffset_height + i]

            if z_val <= -2147483645:  # invalid value
                z_val_mm = None
            else:
                z_val_mm = z_val / 100.0  # um
                z_val_mm /= 1000.0  # mm

            # Luminance data
            lumi_val = self.profdata[addressOffset_lumi + i]

            profile_data[i][0] = x_val_mm
            profile_data[i][1] = z_val_mm
            profile_data[i][2] = lumi_val
        
        profile_data = np.array(profile_data)

        return profile_data

    def what_number_have_last_acquired_profile(self):
        # Change according to your controller settings.
        xpointNum = 3200            # Number of X points per one profile.
        withLumi = 1                # 1: luminance data exists, 0: not exists.

        # Specifies the position, etc. of the profiles to get.
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.req.byTargetBank = 0x0      # 0: active bank
        self.req.byPositionMode = 0x0    # 0: from current position
        self.req.dwGetProfileNo = 0x0    # use when position mode is "POSITION_SPEC"
        self.req.byGetProfileCount = 1   # the number of profiles to read.
        self.req.byErase = 0             # 0: Do not erase

        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()

        self.profinfo = LJXAwrap.LJX8IF_PROFILE_INFO()

        # Calculate the buffer size to store the received profile data.
        self.dataSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        self.dataSize += ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        self.dataSize += ctypes.sizeof(ctypes.c_uint) * xpointNum * (1 + withLumi)
        self.dataSize *= self.req.byGetProfileCount

        dataNumIn4byte = int(self.dataSize / ctypes.sizeof(ctypes.c_uint))
        self.profdata = (ctypes.c_int * dataNumIn4byte)()

        # Send command.
        self.res = LJXAwrap.LJX8IF_GetProfile(self.deviceId,
                                            self.req,
                                            self.rsp,
                                            self.profinfo,
                                            self.profdata,
                                            self.dataSize)

        self.get_logger().info("LJXAwrap.LJX8IF_GetProfile:" + str(hex(self.res)))
        if self.res != 0:
            self.get_logger().info("No profile available")
            return -1
        else:
            return self.rsp.dwGetTopProfileNo

    def get_oldest_profile(self,with_suppression = True):
        # Change according to your controller settings.
        xpointNum = 3200            # Number of X points per one profile.
        withLumi = 1                # 1: luminance data exists, 0: not exists.

        # Specifies the position, etc. of the profiles to get.
        self.req = LJXAwrap.LJX8IF_GET_PROFILE_REQUEST()
        self.req.byTargetBank = 0x0      # 0: active bank
        self.req.byPositionMode = 0x1    # 1: from oldest position
        self.req.dwGetProfileNo = 0x0    # use when position mode is "POSITION_SPEC"
        self.req.byGetProfileCount = 1   # the number of profiles to read.
        self.req.byErase = 1             # 0: Do not erase

        self.rsp = LJXAwrap.LJX8IF_GET_PROFILE_RESPONSE()

        self.profinfo = LJXAwrap.LJX8IF_PROFILE_INFO()

        # Calculate the buffer size to store the received profile data.
        self.dataSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        self.dataSize += ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_FOOTER)
        self.dataSize += ctypes.sizeof(ctypes.c_uint) * xpointNum * (1 + withLumi)
        self.dataSize *= self.req.byGetProfileCount

        dataNumIn4byte = int(self.dataSize / ctypes.sizeof(ctypes.c_uint))
        self.profdata = (ctypes.c_int * dataNumIn4byte)()

        # Send command.
        self.res = LJXAwrap.LJX8IF_GetProfile(self.deviceId,
                                            self.req,
                                            self.rsp,
                                            self.profinfo,
                                            self.profdata,
                                            self.dataSize)

        self.get_logger().info("LJXAwrap.LJX8IF_GetProfile:" + str(hex(self.res)))

        if self.res != 0:
            self.get_logger().info("Failed to get profile.")
            sys.exit()

        headerSize = ctypes.sizeof(LJXAwrap.LJX8IF_PROFILE_HEADER)
        addressOffset_height = int(headerSize / ctypes.sizeof(ctypes.c_uint))
        addressOffset_lumi = addressOffset_height + self.profinfo.wProfileDataCount

        profile_data = [[None, None, None] for i in range(self.profinfo.wProfileDataCount)]

        for i in range(self.profinfo.wProfileDataCount):
            # Conver X data to the actual length in millimeters
            x_val_mm = (self.profinfo.lXStart + self.profinfo.lXPitch * i) / 100.0  # um
            x_val_mm /= 1000.0  # mm

            # Conver Z data to the actual length in millimeters
            z_val = self.profdata[addressOffset_height + i]

            if z_val <= -2147483645:  # invalid value
                z_val_mm = None
            else:
                z_val_mm = z_val / 100.0  # um
                z_val_mm /= 1000.0  # mm

            # Luminance data
            lumi_val = self.profdata[addressOffset_lumi + i]

            profile_data[i][0] = x_val_mm
            profile_data[i][1] = z_val_mm
            profile_data[i][2] = lumi_val
        
        profile_data = np.array(profile_data)

        return profile_data

    def flush_buffer_from_profiles(self):
        LJXAwrap.LJX8IF_ClearMemory(self.deviceId)
        return 0

    def control_service_routine(self, request, response):

        command = request.order

        if command == "laser_on":
            self.enable_laser()
            self.get_logger().info("Laser ON")
            response.validation = True
        elif command == "laser_off":
            self.disable_laser()
            self.get_logger().info("Laser OFF")

            response.validation = True
        elif command == "buffer_flush":
            self.flush_buffer_from_profiles()
            self.get_logger().info("Buffer Flushed")
            response.validation = True
        else:
            response.validation = False


        return response

    def callback_s_a(self, p_header, p_height, p_lumi, luminance_enable, xpointnum, profnum, notify, user):
        if (notify == 0) or (notify == 0x10000):
            if profnum != 0:
                if self.image_available is False:
                    for i in range(xpointnum * profnum):
                        self.z_val[i + (xpointnum * profnum * nb_measure)] = p_height[i]
                        if luminance_enable == 1:
                            self.lumi_val[i + (xpointnum * profnum * nb_measure)] = p_lumi[i]
                    self.ysize_acquired = profnum
                    self.image_available = True
        return

    def get_all_available_profiles_fast_transmission(self,nombre):
        self.image_available = False
        self.ysize_acquired = 0
        self.z_val = []
        self.ysize = nombre
        self.lumi_val = []

        # Initialize Hi-Speed Communication
        self.my_callback_s_a = LJXAwrap.LJX8IF_CALLBACK_SIMPLE_ARRAY(self.callback_s_a)

        self.res = LJXAwrap.LJX8IF_InitializeHighSpeedDataCommunicationSimpleArray(
            self.deviceId,
            self.ethernetConfig,
            self.HighSpeedPortNo,
            self.my_callback_s_a,
            self.ysize,
            0)

        if self.res != 0:
            sys.exit()

        # PreStart Hi-Speed Communication
        self.req = LJXAwrap.LJX8IF_HIGH_SPEED_PRE_START_REQ()
        self.req.bySendPosition = 2
        self.profinfo = LJXAwrap.LJX8IF_PROFILE_INFO()

        self.res = LJXAwrap.LJX8IF_PreStartHighSpeedDataCommunication(
            self.deviceId,
            self.req,
            self.profinfo)

        if self.res != 0:
            sys.exit()

        # allocate the memory
        self.xsize = self.profinfo.wProfileDataCount
        self.z_val = [None] * self.xsize * self.ysize
        self.lumi_val = [None] * self.xsize * self.ysize

        # Start Hi-Speed Communication
        image_available = False
        res = LJXAwrap.LJX8IF_StartHighSpeedDataCommunication(self.deviceId)
        print("LJXAwrap.LJX8IF_StartHighSpeedDataCommunication:", hex(res))
        if res != 0:
            print("\nExit the program.")
            sys.exit()

        LJXAwrap.LJX8IF_StartMeasure(self.deviceId)

        test = True
        timeout_sec = 10
        start_time = time.time()
        while test == True:
            if image_available:
                test = False
            if time.time() - start_time > timeout_sec:
                test = False

        self.res = LJXAwrap.LJX8IF_StopHighSpeedDataCommunication(self.deviceId)
        self.res = LJXAwrap.LJX8IF_FinalizeHighSpeedDataCommunication(self.deviceId)
        return 0

    def profile_saving_service_routine(self, request, response):
        self.disable_laser()
        filename_where_to_save = request.filenametosave

        number_of_profiles = self.what_number_have_last_acquired_profile() + 1

        # Mode ligne par ligne
        #profile_list = [None]*number_of_profiles
        #for i in range(0,number_of_profiles):
        #    profile_list[i] = self.get_profile_number(i)

        self.get_all_available_profiles_fast_transmission(number_of_profiles)

        profiles = np.array(self.z_val)
        self.get_logger().info("taile du tableau sauvegarde : " + str(profiles.shape))
        np.save(filename_where_to_save,profiles)
        self.enable_laser()
        response.confirmation = True
        return response

if __name__ == '__main__':
    rclpy.init()
    keyence_control_node = KeyenceControlNode()
    keyence_control_node.select_job(7)
    rclpy.spin(keyence_control_node)
    rclpy.shutdown()

