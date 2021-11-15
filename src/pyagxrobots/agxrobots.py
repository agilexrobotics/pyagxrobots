#!/usr/bin/env python3
# coding=UTF-8

import sys
import os
import asyncio
import can
import time
import threading
import ctypes
import struct
# import can_msgs
# import rospy

from datetime import datetime
from can import Message
from socket import MsgFlag
from typing import ByteString, Set, Type
from can import bus, interface

from datetime import date
from sqlite3.dbapi2 import Date

import pyagxrobots.UGVConfigMsg as UGVBaseMsg

# Type of current operating system
# posix： Linux
# nt: Windows
# java:

os_is_nt = os.name == 'nt'
#Linux and Mac are both part of the posix standard
# posix: Portable API for Unix-like operating systems
os_is_posix = os.name == 'posix'


class CanMsgsGet:
    def __init__(self, capacity=1024 * 4):
        self.size = 0
        self.rear = 0
        self.front = 0
        self.array = capacity

    def CanMsgsProcess(self, msg):
        """
        Process Can bus data according to ID
        """

        msg = msg
        if (msg.arbitration_id == UGVBaseMsg.CanID.SYSTEM_STATE_ID):
            vehicle_state = int(msg.data[0])
            UGVBaseMsg.SetVehicleState(vehicle_state)
            control_mode = msg.data[1]
            UGVBaseMsg.SetControlMode(control_mode)
            battery_voltage = float((msg.data[2] & 0xff) << 8
                                    | msg.data[3]) / 10
            UGVBaseMsg.SetBatteryVoltage(battery_voltage)
            error_code = msg.data[5]
            UGVBaseMsg.SetErrorCode(error_code)
            count_num = msg.data[7]

            # print(
            #     'vehicle_state:%s control_mode:%s battery_voltage:%s error_code:%s  count_num:%s'
            #     % (vehicle_state, control_mode, battery_voltage, error_code,
            #        count_num))
        elif (msg.arbitration_id == UGVBaseMsg.CanID.MOTION_STATE_ID):
            linear_velocity_get = ctypes.c_int16((msg.data[0] & 0xff) << 8
                                                 | msg.data[1])

            linear_velocity = float(linear_velocity_get.value / 1000)
            # print(int(linear_velocity))
            UGVBaseMsg.SetLinearVelocity(linear_velocity)
            angular_velocity_get = ctypes.c_int16((msg.data[2] & 0xff) << 8
                                                  | msg.data[3])
            angular_velocity = float(angular_velocity_get.value / 1000)
            UGVBaseMsg.SetAngularVelocity(angular_velocity)
            lateral_velocity_get = ctypes.c_int16((msg.data[4] & 0xff) << 8
                                                  | msg.data[5])
            lateral_velocity = float(lateral_velocity_get.value / 1000)
            UGVBaseMsg.SetLateralVelocity(lateral_velocity)
            steering_angle_get = ctypes.c_int16((msg.data[6] & 0xff) << 8
                                                | msg.data[7])
            steering_angle = float(steering_angle_get.value / 1000)
            UGVBaseMsg.SetSteeringAngle(steering_angle)
            # print(msg)

            # print(msg)
            # print(
            #     'linear_velocity:%s angular_velocity:%s lateral_velocity:%s steering_angle:%s '
            #     % (linear_velocity, angular_velocity, lateral_velocity,
            #        steering_angle))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR1_HS_STATE_ID):
            rpm1 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm1(rpm1)
            current1 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent1(current1)
            pulse_count1 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount1(pulse_count1)
            # print('rpm1:%s current1:%s pulse_count1:%s ' %
            #       (rpm1, current1, pulse_count1))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR2_HS_STATE_ID):
            rpm2 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm2(rpm2)
            current2 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent2(current2)
            pulse_count2 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount2(pulse_count2)
            # print('rpm2:%s current2:%s pulse_count2:%s ' %
            #       (rpm2, current2, pulse_count2))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR3_HS_STATE_ID):
            rpm3 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm3(rpm3)
            current3 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent3(current3)
            pulse_count3 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount3(pulse_count3)
            # print('rpm3:%s current3:%s pulse_count3:%s ' %
            #       (rpm3, current3, pulse_count3))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR4_HS_STATE_ID):
            rpm4 = int((msg.data[0] & 0xff) << 8 | msg.data[1])
            UGVBaseMsg.SetRpm4(rpm4)
            current4 = float((msg.data[2] & 0xff) << 8 | msg.data[3]) * 0.1
            UGVBaseMsg.SetCurrent4(current4)
            pulse_count4 = int((msg.data[4] & 0xff) << 24
                               | (msg.data[5] & 0xff) << 16
                               | (msg.data[6] & 0xff) << 8
                               | msg.data[7])
            UGVBaseMsg.SetPulseCount4(pulse_count4)

            # print('rpm4:%s current4:%s pulse_count4:%s ' %
            #       (rpm4, current4, pulse_count4))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR1_LS_STATE_ID):
            driver1_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver1Voltage(driver1_voltage)
            driver1_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver1Temp(driver1_temp)
            motor1_temp = msg.data[4]
            UGVBaseMsg.SetMotor1Temp(motor1_temp)
            driver1_state = msg.data[5]
            UGVBaseMsg.SetDriver1State(driver1_state)
            # print(
            #     'driver_voltage1:%s driver_temp1:%s motor_temp%s driver_state1:%s '
            #     % (driver1_voltage, driver1_temp, motor1_temp, driver1_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR2_LS_STATE_ID):
            driver2_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver2Voltage(driver2_voltage)
            driver2_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver2Temp(driver2_temp)
            motor2_temp = msg.data[4]
            UGVBaseMsg.SetMotor2Temp(motor2_temp)
            driver2_state = msg.data[5]
            UGVBaseMsg.SetDriver2State(driver2_state)
            # print(
            #     'driver_voltage2:%s driver_temp2:%s  motor_temp2:%s driver_state2:%s '
            #     % (driver2_voltage, driver2_temp, motor2_temp, driver2_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR3_LS_STATE_ID):
            driver3_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver3Voltage(driver3_voltage)
            driver3_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver3Temp(driver3_temp)
            motor3_temp = msg.data[4]
            UGVBaseMsg.SetMotor3Temp(motor3_temp)
            driver3_state = msg.data[5]
            UGVBaseMsg.SetDriver3State(driver3_state)
            # print(
            #     'driver_voltage3:%s driver_temp3:%s  motor_temp3:%s driver_state3:%s '
            #     % (driver3_voltage, driver3_temp, motor3_temp, driver3_state))
        elif (msg.arbitration_id == UGVBaseMsg.CanID.ACTUATOR4_LS_STATE_ID):
            driver4_voltage = float((msg.data[0] & 0xff) << 8
                                    | msg.data[1]) * 0.1
            UGVBaseMsg.SetDriver4Voltage(driver4_voltage)
            driver4_temp = int((msg.data[2] & 0xff) << 8 | msg.data[3])
            UGVBaseMsg.SetDriver4Temp(driver4_temp)
            motor4_temp = msg.data[4]
            UGVBaseMsg.SetMotor4Temp(motor4_temp)
            driver4_state = msg.data[5]
            UGVBaseMsg.SetDriver4State(driver4_state)
            # print(
            #     'driver_voltage4:%s driver_temp4:%s  motor_temp:%s driver_state4:%s '
            #     % (driver4_voltage, driver4_temp, motor4_temp, driver4_state))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.LIGHT_STATE_ID):
            light_cmd_ctrl = (msg.data[0])
            UGVBaseMsg.SetLightCmdCtrl(light_cmd_ctrl)
            front_mode = (msg.data[1])
            UGVBaseMsg.SetFrontMode(front_mode)
            front_custom = (msg.data[2])
            UGVBaseMsg.SetFrontCustom(front_custom)
            rear_mode = (msg.data[3])
            UGVBaseMsg.SetRearMode(rear_mode)
            rear_custom = (msg.data[4])
            UGVBaseMsg.SetRearCustom(rear_custom)

            # print(
            #     'enable_cmd_ctrl:%s front_mode:%s  front_custom:%s rear_mode:%s  rear_custom:%s'
            #     % (light_cmd_ctrl, front_mode, front_custom, rear_mode,
            #        rear_custom))
        elif (msg.arbitration_id == UGVBaseMsg.CanID.VERSION_RESPONSE_ID):
            control_hardware_version = int((msg.data[0] & 0xff) << 8
                                           | msg.data[1])
            UGVBaseMsg.SetControlHardwareVersion(control_hardware_version)
            actuaror_hardware_version = int((msg.data[2] & 0xff) << 8
                                            | msg.data[3])
            UGVBaseMsg.SetActuarorHardwareVersion(actuaror_hardware_version)
            control_software_version = int((msg.data[4] & 0xff) << 8
                                           | msg.data[5])
            UGVBaseMsg.SetControlSoftwareVersion(control_software_version)
            actuaror_software_version = int((msg.data[6] & 0xff) << 8
                                            | msg.data[7])
            UGVBaseMsg.SetActuarorSoftwareVersion(actuaror_software_version)
            # print(
            #     'control_hardware_version: %s actuaror_hardware_version: %s' %
            #     (control_hardware_version, actuaror_hardware_version))
            # print('control_software_version:%s actuaror_software_version:%s ' %
            #       (control_software_version, actuaror_software_version))

        elif (msg.arbitration_id == UGVBaseMsg.CanID.ODOMETRY_ID):
            left_wheel_get = ctypes.c_int((msg.data[0] & 0xff) << 24
                                          | (msg.data[1] & 0xff) << 16
                                          | (msg.data[2] & 0xff) << 8
                                          | msg.data[3])
            left_wheel = left_wheel_get.value
            UGVBaseMsg.SetLeftWheel(left_wheel)
            right_wheel_get = ctypes.c_int((msg.data[4] & 0xff) << 24
                                           | (msg.data[5] & 0xff) << 16
                                           | (msg.data[6] & 0xff) << 8
                                           | msg.data[7])
            # print('left_wheel: %s right_wheel: %s ' %
            #       (left_wheel, right_wheel))
            right_wheel = right_wheel_get.value
            UGVBaseMsg.SetRightWheel(right_wheel)
            # print(msg)
        elif (msg.arbitration_id == UGVBaseMsg.CanID.RC_STATE_ID):
            sws = msg.data[0]
            UGVBaseMsg.SetSws(sws)
            stick_right_v = msg.data[1]
            UGVBaseMsg.SetStickRightV(stick_right_v)
            stick_right_h = msg.data[2]
            UGVBaseMsg.SetStickRightH(stick_right_h)
            stick_left_v = msg.data[3]
            UGVBaseMsg.SetStickLeftV(stick_left_v)
            stick_left_h = msg.data[4]
            UGVBaseMsg.SetStickLeftH(stick_left_h)
            var_a = msg.data[5]
            UGVBaseMsg.SetVarA(var_a)


class DeviceCan:
    """
    
    """
    def __init__(self, bustype='', channel='', bitrate=0):
        self.bustype = str(bustype)
        self.channel = str(channel)
        self.bitrate = int(bitrate)
        UGVBaseMsg._init()
        self.canport = can.interface.Bus(bustype=self.bustype,
                                         channel=self.channel,
                                         bitrate=self.bitrate)

        # with self.canport as server:
        with can.interface.Bus(bustype=self.bustype,
                               channel=self.channel,
                               bitrate=self.bitrate) as client:
            # stop_event = threading.Event()
            t_receive = threading.Thread(target=self.EnableAsynsCan)
            t_receive.start()
            # self.EnableAsynsCan(client)

            try:
                while (UGVBaseMsg.GetLen() < UGVBaseMsg.CanMsgLen.CAN_MSG_LEN):
                    self.msg = Message(
                        arbitration_id=UGVBaseMsg.CanID.VERSION_REQUEST_ID,
                        data=[0x01],
                        is_extended_id=False)
                    self.CanSend(self.msg)
                    time.sleep(0.1)

            except (KeyboardInterrupt, SystemExit):
                pass
            # stop_event.set()
            time.sleep(0.5)

    async def CanReceive(self, bus, stop_event):
        """The loop for receiving."""
        print("Start receiving messages")
        SlectMsg = CanMsgsGet()
        while not stop_event.is_set():

            self.rx_msg = await bus.recv(0.002)
            if self.rx_msg is not None:
                SlectMsg.CanMsgsProcess(self.rx_msg)
                print("rx: {}".format(self.rx_msg))

        await bus.recv()
        print("Stopped receiving messages")
        if stop_event.is_set() == False:
            stop_event.set()

    def CanGet(self):
        self.canport = can.ThreadSafeBus(interface=self.bustype,
                                         channel=self.channel,
                                         bitrate=self.bitrate)

        msg = self.canport.recv(0.002)
        print(msg)
        return msg

    def EnableAsynsCan(self):

        LOOP = asyncio.new_event_loop()
        asyncio.set_event_loop(LOOP)
        # Run until main coroutine finishes
        LOOP.run_until_complete(self.AsynsCan())

    def ListenerMessage(self, msg):
        SlectMsg = CanMsgsGet()
        SlectMsg.CanMsgsProcess(msg)

    async def AsynsCan(self):
        bus = can.Bus(interface=self.bustype,
                      channel=self.channel,
                      bitrate=self.bitrate)

        reader = can.BufferedReader()
        logger = can.Logger('logfile.asc')
        listeners = [
            self.ListenerMessage,
            reader,  # AsyncBufferedReader() listener
            logger  # Regular Listener object
        ]
        loop = asyncio.get_event_loop()
        notifier = can.Notifier(bus, listeners, loop=loop)
        while 1:
            msg = reader.get_message()
            await asyncio.sleep(0.2)

        notifier.stop()

    def CanDataUpdate(self):
        msg = self.CanGet()
        SlectMsg = CanMsgsGet()
        if msg is None:
            print('time occerred,no message')
        else:

            SlectMsg.CanMsgsProcess(msg)

    def CanSend(self, msg):

        self.canport = can.ThreadSafeBus(interface=self.bustype,
                                         channel=self.channel,
                                         bitrate=self.bitrate)
        with self.canport as bus:
            try:
                bus.send(msg)
            except can.CanError:
                print("Message NOT sent")


class UGVCan:
    def __init__(self, bustype=None, channel=None, bitrate=None):
        can_device_kwargs = {}
        if bitrate is None:
            can_device_kwargs['bitrate'] = 500000
        else:
            can_device_kwargs['bitrate'] = bitrate

        if bustype is None:
            can_device_kwargs['bustype'] = "socketcan"
        else:
            can_device_kwargs['bustype'] = bustype

        if channel is None:
            can_device_kwargs['channel'] = "can0"
        else:
            default_channel = channel
            can_device_kwargs['channel'] = default_channel

        self.candevice = DeviceCan(**can_device_kwargs)

    def Get(self):
        return self.candevice.CanGet()

    def Send(self, msg):
        self.candevice.CanSend(msg)


class UGV:
    """
        EnableCANCtrl()
        SendVersionRequest()
        SendErrorClearByte()
        EnableLightCtrl()
        DisableLightCtrl()
        LightFrontMode()
        SendLinerVelocity()
        SendAngularVelocity()

        GetLightMode()        
        GetSysVersion()
        GetLeftWheelOdem()
        GetRightWheelOdem()
        GetLinerVelocity()
        GetAngularVelocity()
        GetErrorCode()
    
    """
    def __init__(self, *device_args, **device_kwargs):
        self.device = None
        can_device_init_fn = UGVCan.__init__
        args_names = can_device_init_fn.__code__.co_varnames[:
                                                             can_device_init_fn
                                                             .__code__.
                                                             co_argcount]
        args_dict = dict(zip(args_names, device_args))
        args_dict.update(device_kwargs)

        self.device = UGVCan(**args_dict)

    UGVBaseMsg._init()

    def SendMsg(self, msg):
        self.device.Send(msg)

    def GetMsg(self):
        return self.device.Get()

    def SendVersionRequest(self):
        """
        Version Information Request
        """
        self.msg = Message(arbitration_id=UGVBaseMsg.CanID.VERSION_REQUEST_ID,
                           data=[0x01],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    def EnableCANCtrl(self):
        """
        控制底盘处于Can控制模式，需将遥控SWB拨到最上方
        Control the chassis in Can control mode, you need to dial the remote control SWB to the top
        """
        self.msg = Message(arbitration_id=UGVBaseMsg.CanID.CTRL_MODE_CONFIG_ID,
                           data=[0x01],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    def SendErrorClearByte(self, clear_byte):
        """
        clear the chassis motro error 

        clear_byte:0-4
            0 clear all 
            1 only clear motro1
            2 only clear motro2
            3 only clear motro3
            4 only clear motro4
        """
        if clear_byte > 4 | clear_byte < 0:
            clear_byte = 0
        else:
            pass
        self.msg = can.Message(
            arbitration_id=UGVBaseMsg.CanID.STATE_RESET_CONFIG_ID,
            data=[clear_byte],
            is_extended_id=False)
        self.SendMsg(self.msg)

    def EnableLightCtrl(self):
        """
        Enable light control
        """
        get_data = UGVBaseMsg.GetLightCmdCtrl()
        if (get_data != None):
            set_data = ((get_data) | 1)
            UGVBaseMsg.SetLightCmdCtrl(set_data)

            self.msg = Message(
                arbitration_id=UGVBaseMsg.CanID.LIGHT_COMMAND_ID,
                data=[
                    UGVBaseMsg.GetLightCmdCtrl(),
                    UGVBaseMsg.GetFrontMode(),
                    UGVBaseMsg.GetFrontCustom()
                ],
                is_extended_id=False)

            self.SendMsg(self.msg)

    def DisableLightCtrl(self):
        """
         Disable light control
        """
        data = UGVBaseMsg.GetLightCmdCtrl()
        data = data & 0xfe
        UGVBaseMsg.SetLightCmdCtrl(data)
        self.msg = Message(arbitration_id=UGVBaseMsg.CanID.LIGHT_COMMAND_ID,
                           data=[
                               UGVBaseMsg.GetLightCmdCtrl(),
                               UGVBaseMsg.GetFrontMode(),
                               UGVBaseMsg.GetFrontCustom(),
                           ],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    def LightFrontMode(self, mode, bright=None):
        """
        mode:0 Often shut
        mode:1 Normally open
        mode:2 Breathing lamp
        mode:3 Custom
        bright:        0-100       Note：Must be in mode 3
        """
        if bright != None:
            if bright>=100:
                bright=100
            elif bright<=0:
                bright=0
            if mode == 0x03:
                UGVBaseMsg.SetFrontCustom(bright)
            else:
                print("please in mode 3")

        self.msg = Message(arbitration_id=UGVBaseMsg.CanID.LIGHT_COMMAND_ID,
                           data=[
                               UGVBaseMsg.GetLightCmdCtrl(),
                               mode,
                               UGVBaseMsg.GetFrontCustom()
                           ],
                           is_extended_id=False)
        self.SendMsg(self.msg)

    # def SendSpeedAngular(self, speed, angular):
    #     """
    #     send speed and angular data to chassis
    #     speed :-3~3 m/s
    #     angular:-2.523~2.523  rad/s
    #     """
    #     if (((speed * 1000) > 3000) | ((speed * 1000) < -3000) |
    #         (speed == None)):
    #         speed = 0
    #         print('Speed must be between -3 and 3m/s ')
    #     if (((angular * 1000) > 2523) | ((angular * 1000) < -2523) |
    #         (angular == None)):
    #         angular = 0
    #         print('Angular must be between -2.523 and 2.523 rad/s ')

    #     msg = Message(arbitration_id=UGVBaseMsg.CanID.MOTION_COMMAND_ID,
    #                   data=[(int(speed * 1000)) >> 8 & 0xff,
    #                         (int(speed * 1000)) & 0xff,
    #                         (int(angular * 1000)) >> 8 & 0xff,
    #                         (int(angular * 1000)) & 0xff],
    #                   is_extended_id=False)
    #     self.SendMsg(msg)

    def SendLinerVelocity(self, speed):
        """
        send speed  data to chassis
        speed :-3.000~3.000 mm/s
    
        """
        if (((speed * 1000) > 3000) | ((speed * 1000) < -3000) |
            (speed == None)):
            speed = 0
            print('Speed must be between -3 and 3m/s ')
        UGVBaseMsg.SetLinearVelocity(speed)

        msg = Message(arbitration_id=UGVBaseMsg.CanID.MOTION_COMMAND_ID,
                      data=[
                          (int(speed * 1000) >> 8) & 0xff,
                          int(speed * 1000) & 0xff,
                      ],
                      is_extended_id=False)
        self.SendMsg(msg)

    def SendAngularVelocity(self, angular):
        """
        send  angular data to chassis
        angular :-2.523~2.523 rad/s
        """
        if (((angular * 1000) > 2523) | ((angular * 1000) < -2523) |
            (angular == None)):
            angular = 0
            print('Angular must be between -2.523 and 2.523 rad/s ')
        UGVBaseMsg.SetLinearVelocity(angular)

        msg = Message(arbitration_id=UGVBaseMsg.CanID.MOTION_COMMAND_ID,
                      data=[
                          0, 0, (int(angular * 1000) >> 8) & 0xff,
                          int(angular * 1000) & 0xff
                      ],
                      is_extended_id=False)
        self.SendMsg(msg)

    # def  MotionCommand(self,msg):
    #     if (msg.id == 0X111):
    #         can.Message.arbitration_id=0X111
    #         msg = can.Message(arbitration_id=0x421, data=[0x01], is_extended_id=False)

    #         SendOne(msg)
    #     else:print("command_id is err")

    # def  BrakingCommand(self,msg):
    #     if (msg.id == 0X131):
    #         can.Message.arbitration_id=msg.id
    #         can.Message.data=msg.data
    #         SendOne(can.Message)
    #     else:print("command_id is err")

    # def  MotionCommand(self,msg):
    #     if (msg.id == 0X141):
    #         can.Message.arbitration_id=msg.id
    #         can.Message.data=msg.data
    #         SendOne(can.Message)
    #     else:print("command_id is err")

    def GetLightMode(self):
        return (UGVBaseMsg.GetLightCmdCtrl(), UGVBaseMsg.GetFrontMode(),
                UGVBaseMsg.GetFrontCustom())

    def GetSysVersion(self):
        self.SendVersionRequest()
        return (UGVBaseMsg.GetControlHardwareVersion(),
                UGVBaseMsg.GetActuarorHardwareVersion(),
                UGVBaseMsg.GetControlSoftwareVersion(),
                UGVBaseMsg.GetActuarorSoftwareVersion())

    def GetLeftWheelOdem(self):
        return UGVBaseMsg.GetLeftWheel()

    def GetRightWheelOdem(self):
        return UGVBaseMsg.GetRightWheel()

    def GetLinerVelocity(self):
        return UGVBaseMsg.GetLinearVelocity()

    def GetAngularVelocity(self):
        return UGVBaseMsg.GetAngularVelocity()

    def GetErrorCode(self):
        return UGVBaseMsg.GetErrorCode()
