#!/usr/bin/env python
# coding=UTF-8


def _init():
    global _global_dict
    _global_dict = {}


def SetValue(name, value):
    _global_dict[name] = value


def GetValue(name):
    return _global_dict.get(name, 0)


def GetLen():
    return len(_global_dict)


def SetVehicleState(vehicle_state):
    SetValue('vehicle_state', vehicle_state)


def GetVehicleState():
    return GetValue('vehicle_state')


def SetControlMode(control_mode):
    SetValue('control_mode', control_mode)


def GetControlMode():
    return GetValue('control_mode')


def SetBatteryVoltage(battery_voltage):
    SetValue('battery_voltage', battery_voltage)


def GetBatteryVoltage():
    return GetValue('battery_voltage')


def SetErrorCode(error_code):
    SetValue('error_code', error_code)


def GetErrorCode():
    return GetValue('error_code')


def SetLinearVelocity(linear_velocity):
    SetValue('linear_velocity', linear_velocity)


def GetLinearVelocity():
    return GetValue('linear_velocity')


def SetAngularVelocity(angular_velocity):
    SetValue('angular_velocity', angular_velocity)


def GetAngularVelocity():
    return GetValue('angular_velocity')


def SetLateralVelocity(lateral_velocity):
    SetValue('lateral_velocity', lateral_velocity)


def GetLateralVelocity():
    return GetValue('lateral_velocity')


def SetSteeringAngle(steering_angle):
    SetValue('steering_angle', steering_angle)


def GetSteeringAngle():
    return GetValue('steering_angle')


def SetRpm1(rpm1):
    SetValue('rpm1', rpm1)


def GetRpm1():
    return GetValue('rpm1')


def SetRpm2(rpm2):
    SetValue('rpm2', rpm2)


def GetRpm2():
    return GetValue('rpm2')


def SetRpm3(rpm3):
    SetValue('rpm3', rpm3)


def GetRpm3():
    return GetValue('rpm3')


def SetRpm4(rpm4):
    SetValue('rpm4', rpm4)


def GetRpm4():
    return GetValue('rpm4')


def SetCurrent1(current1):
    SetValue('current1', current1)


def GetCurrent1():
    return GetValue('current1')


def SetCurrent2(current2):
    SetValue('current2', current2)


def GetCurrent2():
    return GetValue('current2')


def SetCurrent3(current3):
    SetValue('current3', current3)


def GetCurrent3():
    return GetValue('current3')


def SetCurrent4(current4):
    SetValue('current4', current4)


def GetCurrent4():
    return GetValue('current4')


def SetPulseCount1(pulse_count1):
    SetValue('pulse_count1', pulse_count1)


def GetPulseCount1():
    return GetValue('pulse_count1')


def SetPulseCount2(pulse_count2):
    SetValue('pulse_count2', pulse_count2)


def GetPulseCount2():
    return GetValue('pulse_count2')


def SetPulseCount3(pulse_count3):
    SetValue('pulse_count3', pulse_count3)


def GetPulseCount3():
    return GetValue('pulse_count3')


def SetPulseCount4(pulse_count4):
    SetValue('pulse_count4', pulse_count4)


def GetPulseCount4():
    return GetValue('pulse_count4')


def SetDriver1Voltage(driver1_voltage):
    SetValue('driver1_voltage', driver1_voltage)


def GetDriver1Voltage():
    return GetValue('driver1_voltage')


def SetDriver1Temp(driver1_temp):
    SetValue('driver1_temp', driver1_temp)


def GetDriver1Temp():
    return GetValue('driver1_temp')


def SetMotor1Temp(motor1_temp):
    SetValue('motor1_temp', motor1_temp)


def GetMotor1Temp():
    return GetValue('motor1_temp')


def SetDriver1State(driver1_state):
    SetValue('driver1_state', driver1_state)


def GetDriver1State():
    return GetValue('driver1_state')


def SetDriver2Voltage(driver2_voltage):
    SetValue('driver2_voltage', driver2_voltage)


def GetDriver2Voltage():
    return GetValue('driver2_voltage')


def SetDriver2Temp(driver2_temp):
    SetValue('driver2_temp', driver2_temp)


def GetDriver2Temp():
    return GetValue('driver2_temp')


def SetMotor2Temp(motor2_temp):
    SetValue('motor2_temp', motor2_temp)


def GetMotor2Temp():
    return GetValue('motor2_temp')


def SetDriver2State(driver2_state):
    SetValue('driver2_state', driver2_state)


def GetDriver2State():
    return GetValue('driver2_state')


def SetDriver3Voltage(driver3_voltage):
    SetValue('driver3_voltage', driver3_voltage)


def GetDriver3Voltage():
    return GetValue('driver3_voltage')


def SetDriver3Temp(driver3_temp):
    SetValue('driver_temp', driver3_temp)


def GetDriver3Temp():
    return GetValue('driver3_temp')


def SetMotor3Temp(motor3_temp):
    SetValue('motor3_temp', motor3_temp)


def GetMotor3Temp():
    return GetValue('motor3_temp')


def SetDriver3State(driver3_state):
    SetValue('driver3_state', driver3_state)


def GetDriver3State():
    return GetValue('driver3_state')


def SetDriver4Voltage(driver4_voltage):
    SetValue('driver4_voltage', driver4_voltage)


def GetDriver4Voltage():
    return GetValue('driver4_voltage')


def SetDriver4Temp(driver4_temp):
    SetValue('driver4_temp', driver4_temp)


def GetDriver4Temp():
    return GetValue('driver4_temp')


def SetMotor4Temp(motor4_temp):
    SetValue('motor4_temp', motor4_temp)


def GetMotor4Temp():
    return GetValue('motor4_temp')


def SetDriver4State(driver4_state):
    SetValue('driver_state', driver4_state)


def GetDriver4State():
    return GetValue('driver4_state')


def SetLightCmdCtrl(light_cmd_ctrl):
    SetValue('light_cmd_ctrl', light_cmd_ctrl)


def GetLightCmdCtrl():
    return GetValue('light_cmd_ctrl')


def SetFrontMode(front_mode):
    SetValue('front_mode', front_mode)


def GetFrontMode():
    return GetValue('front_mode')


def SetFrontCustom(front_custom):
    SetValue('front_custom', front_custom)


def GetFrontCustom():
    return GetValue('front_custom')


def SetRearMode(rear_mode):
    SetValue('rear_mode', rear_mode)


def GetRearMode():
    return GetValue('rear_mode')


def SetRearCustom(rear_custom):
    SetValue('rear_custom', rear_custom)


def GetRearCustom():
    return GetValue('rear_custom')


def SetControlHardwareVersion(control_hardware_version):
    SetValue('control_hardware_version', control_hardware_version)


def GetControlHardwareVersion():
    return GetValue('control_hardware_version')


def SetActuarorHardwareVersion(actuaror_hardware_version):
    SetValue('actuaror_hardware_version', actuaror_hardware_version)


def GetActuarorHardwareVersion():
    return GetValue('actuaror_hardware_version')


def SetControlSoftwareVersion(control_software_version):
    SetValue('control_software_version', control_software_version)


def GetControlSoftwareVersion():
    return GetValue('control_software_version')


def SetActuarorSoftwareVersion(actuaror_software_version):
    SetValue('actuaror_software_version', actuaror_software_version)


def GetActuarorSoftwareVersion():
    return GetValue('actuaror_software_version')


def SetLeftWheel(left_wheel):
    SetValue('left_wheel', left_wheel)


def GetLeftWheel():
    return GetValue('left_wheel')


def SetRightWheel(right_wheel):
    SetValue('right_wheel', right_wheel)


def GetRightWheel():
    return GetValue('right_wheel')


def SetSws(sws):
    SetValue('sws', sws)


def GetSws():
    return GetValue('sws')


def SetStickRightV(stick_right_v):
    SetValue('stick_right_v', stick_right_v)


def GetStickRightV():
    return GetValue('stick_right_v')


def SetStickRightH(stick_right_h):
    SetValue('stick_right_h', stick_right_h)


def GetStickRightH():
    return GetValue('stick_right_h')


def SetStickLeftV(stick_left_v):
    SetValue('stick_left_v', stick_left_v)


def GetStickLeftV():
    return GetValue('stick_left_v')


def SetStickLeftH(stick_left_h):
    SetValue('stick_left_h', stick_left_h)


def GetStickLeftH():
    return GetValue('stick_left_h')


def SetVarA(var_a):
    SetValue('var_a', var_a)


def GetVarA():
    return GetValue('var_a')


def SetBaseNeutral(base_neutral):
    SetValue('base_neutral', base_neutral)


def GetBaseNeutral():
    return GetValue('base_neutral')


class CanMsgLen(object):
    CAN_MSG_LEN = 53


class CanIDV2(object):
    # // control group: 0x1
    MOTION_COMMAND_ID = 0X111
    LIGHT_COMMAND_ID = 0X121
    BRAKING_COMMAND_ID = 0X131
    SET_MOTION_MODE_ID = 0X141

    # // state feedback group: 0x2
    SYSTEM_STATE_ID = 0X211
    MOTION_STATE_ID = 0X221
    LIGHT_STATE_ID = 0X231
    RC_STATE_ID = 0X241

    # //get catuator hight speed state
    ACTUATOR1_HS_STATE_ID = 0X251
    ACTUATOR2_HS_STATE_ID = 0X252
    ACTUATOR3_HS_STATE_ID = 0X253
    ACTUATOR4_HS_STATE_ID = 0X254
    ACTUATOR5_HS_STATE_ID = 0X255
    ACTUATOR6_HS_STATE_ID = 0X256
    ACTUATOR7_HS_STATE_ID = 0X257
    ACTUATOR8_HS_STATE_ID = 0X258

    # //get actuator low speed state
    ACTUATOR1_LS_STATE_ID = 0X261
    ACTUATOR2_LS_STATE_ID = 0X262
    ACTUATOR3_LS_STATE_ID = 0X263
    ACTUATOR4_LS_STATE_ID = 0X264
    ACTUATOR5_LS_STATE_ID = 0X265
    ACTUATOR6_LS_STATE_ID = 0X266
    ACTUATOR7_LS_STATE_ID = 0X267
    ACTUATOR8_LS_STATE_ID = 0X268

    MOTOR_ANGLE_INFO_ID = 0X271
    MOTOR_SPEED_INFO_ID = 0X281
    CURRENT_CTRL_MODE_ID = 0X291

    # // sensor data group: 0x3
    ODOMETRY_ID = 0X311
    IMU_ACCEL_ID = 0X321
    IMU_GYRO_ID = 0X322
    IMU_EULER_ID = 0X323
    SAFETY_BUMPER_ID = 0X331

    BMS_BASIC_ID = 0X361
    BMS_EXTENDED_ID = 0X362

    # // query/config group: 0x4
    VERSION_REQUEST_ID = 0X411
    VERSION_RESPONSE_ID = 0X41A
    CTRL_MODE_CONFIG_ID = 0X421
    STEER_NEUTRAL_REQUEST_ID = 0X431
    STEER_NEUTRAL_RESPONSE_ID = 0X43A
    STATE_RESET_CONFIG_ID = 0X441


class CanIDV1(object):
    # CAN: control group

    MOTION_COMMAND_ID = 0x130
    LIGHT_COMMAND_ID = 0x140
    VALUE_SET_COMMAND_ID = 0x210

    # CAN: state feedback group
    MOTION_STATE_ID = 0x131
    LIGHT_STATE_ID = 0x141
    SYSTEM_STATE_ID = 0x151

    VALUE_SET_STATE_ID = 0x211

    ACTUATOR1_STATE_ID = 0x200
    ACTUATOR2_STATE_ID = 0x201
    ACTUATOR3_STATE_ID = 0x202
    ACTUATOR4_STATE_ID = 0x203
