import pyagxrobots.UGVConfigMsg as UGVBaseMsg


class MotionCommandMessage(object):
    def GetLinearVelocity(self):
        return float(UGVBaseMsg.GetLinearVelocity())

    def GetAngularVelocity(self):
        # // only valid for differential drivering
        return float(UGVBaseMsg.GetAngularVelocity())

    def GetLateralVelocity(self):
        return float(UGVBaseMsg.GetLateralVelocity())

    def GetSteeringAngle(self):
        # // only valid for ackermann steering
        return float(UGVBaseMsg.GetSteeringAngle())


class LightCommandMessage(object):
    def GetLightCmdCtrl(self):
        return UGVBaseMsg.GetLightCmdCtrl()

    def GetFrontMode(self):
        return UGVBaseMsg.GetFrontMode()

    def GetFrontCustom(self):
        return UGVBaseMsg.GetFrontCustom()

    def GetRearMode(self):
        return UGVBaseMsg.GetRearMode()

    def GetRearCustom(self):
        return UGVBaseMsg.GetRearCustom()


class SystemStateMessage(object):
    def GetVehicleState(self):
        return UGVBaseMsg.GetVehicleState()

    def GetControlMode(self):
        return UGVBaseMsg.GetControlMode()

    def GetBatteryVoltage(self):
        return float(UGVBaseMsg.GetBatteryVoltage())

    def GetErrorCode(self):
        return UGVBaseMsg.GetErrorCode()


class RcStateMessage(object):
    def GetVarA(self):
        return UGVBaseMsg.GetVarA()

    def GetSws(self):
        return UGVBaseMsg.GetSws()

    def GetStickRightV(self):
        return UGVBaseMsg.GetStickRightV()

    def GetStickRightH(self):
        return UGVBaseMsg.GetStickRightH()

    def GetStickLeftV(self):
        return UGVBaseMsg.GetStickLeftV()

    def GetStickLeftH(self):
        return UGVBaseMsg.GetStickLeftH()


class OdometryMessage(object):
    def GetLeftWheel(self):
        return float(UGVBaseMsg.GetLeftWheel())

    def GetRightWheel(self):
        return float(UGVBaseMsg.GetRightWheel())


class ActuatorStateMessageV1(object):
    def __init__(self, motro_id=0):
        self.motro_id = motro_id

    def current(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetCurrent1()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetCurrent2()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetCurrent3()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetCurrent4()

    def rpm(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetRpm1()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetRpm2()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetRpm3()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetRpm4()

    def driver_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetDriver1Temp()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetDriver2Temp()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetDriver3Temp()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetDriver4Temp()

    def motor_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetMotor1Temp()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetMotor2Temp()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetMotor3Temp()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetMotor4Temp()


class ActuatorStateMessageV2(object):
    def __init__(self, motro_id=0):
        self.motro_id = motro_id

    def rpm(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetRpm1()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetRpm2()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetRpm3()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetRpm4()

    def current(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetCurrent1()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetCurrent2()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetCurrent3()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetCurrent4()

    def pulse_count(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetPulseCount1()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetPulseCount2()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetPulseCount3()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetPulseCount4()

    def driver_voltage(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetDriver1Voltage()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetDriver2Voltage()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetDriver3Voltage()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetDriver4Voltage()

    def driver_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetDriver1Temp()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetDriver2Temp()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetDriver3Temp()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetDriver4Temp()

    def motor_temp(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetMotor1Temp()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetMotor2Temp()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetMotor3Temp()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetMotor4Temp()

    def driver_state(self):
        if self.motro_id == 1:
            return UGVBaseMsg.GetDriver1State()
        elif self.motro_id == 2:
            return UGVBaseMsg.GetDriver2State()
        elif self.motro_id == 3:
            return UGVBaseMsg.GetDriver3State()
        elif self.motro_id == 4:
            return UGVBaseMsg.GetDriver4State()


class GetRobotStae(MotionCommandMessage,
                   LightCommandMessage,
                   SystemStateMessage,
                   RcStateMessage,
                   OdometryMessage):
    pass
