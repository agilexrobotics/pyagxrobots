#!/usr/bin/env python3
# coding=UTF-8
import sys
import os

from wrapt.wrappers import transient_function_wrapper

import pyagxrobots.UGVConfigMsg as UGVBaseMsg
import pyagxrobots.agxrobots as agxrobots
from pyagxrobots.agxbase import ActuatorStateMessageV1, GetRobotStae
from pyagxrobots.agxbase import ActuatorStateMessageV2


class BunkerBase(GetRobotStae):
    def __init__(self) :
        self.basetype='bunker'
        self.motronum= 2
        self.bunkerbase=agxrobots.UGV()
        self.baseversion=self.bunkerbase.base_version
        self.max_linear=3.0
        self.max_angular=2.5235
        self.max_lateral=0.0
        self.min_linear=-self.max_linear
        self.min_angular=-self.max_angular
        self.min_lateral=-self.max_lateral
        if self.baseversion==1:
            self.BunkerActuator1=ActuatorStateMessageV1(motro_id=1)
            self.BunkerActuator2=ActuatorStateMessageV1(motro_id=2)
        elif self.baseversion==2:
            self.BunkerActuator1=ActuatorStateMessageV2(motro_id=1)
            self.BunkerActuator2=ActuatorStateMessageV2(motro_id=2)

    def  SetMotionCommand(self,linear_vel,angular_vel):

        if linear_vel>self.max_linear:
            linear_vel=self.max_linear
        elif linear_vel<self.min_linear:
            linear_vel=self.min_linear
        if angular_vel>self.max_angular:
            angular_vel=self.max_angular
        elif angular_vel<self.min_angular:
            angular_vel=self.min_angular
        self.bunkerbase.SetMotionCommand(linear_vel,angular_vel,0,0)
  
        # self.version=   
    def EnableCAN(self):
        if self.baseversion==2:
            self.bunkerbase.EnableCANCtrl()   
       
    
    
   
class HunterBase(GetRobotStae):
    def __init__(self) :
        self.basetype='hunter'
        self.motronum= 2
        self.bunterbase=agxrobots.UGV()
        self.baseversion=self.bunterbase.base_version
        self.max_linear=1.5
        self.max_angular=0.444
        self.max_lateral=0.0
        self.min_linear=-self.max_linear
        self.min_angular=-self.max_angular
        self.min_lateral=-self.max_lateral
        if self.baseversion==1:
            self.HunterActuator1=ActuatorStateMessageV1(motro_id=1)
            self.HunterActuator2=ActuatorStateMessageV1(motro_id=2)
        elif self.baseversion==2:
            self.HunterActuator1=ActuatorStateMessageV2(motro_id=1)
            self.HunterActuator2=ActuatorStateMessageV2(motro_id=2)
    def  SetMotionCommand(self,linear_vel,angular_vel):
        if self.baseversion==1:
            if linear_vel>self.max_linear:
                linear_vel=self.max_linear
            elif linear_vel<self.min_linear:
                linear_vel=self.min_linear
            if angular_vel>self.max_angular:
                angular_vel=self.max_angular
            elif angular_vel<self.min_angular:
                angular_vel=self.min_angular       
        self.bunterbase.SetMotionCommand(linear_vel,0,0,angular_vel)

    def EnableCAN(self):
        if self.baseversion==2:
            self.bunterbase.EnableCANCtrl()   



class RangerBase(GetRobotStae):
    def __init__(self) :
        self.basetype='ranger'
        self.motronum= 4
        self.rangerbase=agxrobots.UGV()
        self.baseversion=self.rangerbase.base_version
        if self.baseversion==1:
            self.RangerActuator1=ActuatorStateMessageV1(motro_id=1)
            self.RangerActuator2=ActuatorStateMessageV1(motro_id=2)
            self.RangerActuator3=ActuatorStateMessageV1(motro_id=3)
            self.RangerActuator4=ActuatorStateMessageV2(motro_id=4)
        elif self.baseversion==2:
            self.RangerActuator1=ActuatorStateMessageV2(motro_id=1)
            self.RangerActuator2=ActuatorStateMessageV2(motro_id=2)
            self.RangerActuator3=ActuatorStateMessageV2(motro_id=3)
            self.RangerActuator4=ActuatorStateMessageV2(motro_id=4)  

    def  SetMotionCommand(self,linear_vel=0.0,lateral_vel=0.0,angular_vel=0.0,steer_angle=0.0):
        
        self.rangerbase.SetMotionCommand(linear_vel,angular_vel,lateral_vel,steer_angle)
    def SetLightCommand(self,f_mode,f_value,r_mode,r_value):
        self.rangerbase.SetLightCommand(f_mode,f_value,r_mode,r_value)   

       # self.version= 
    def EnableCAN(self):
        if self.baseversion==2:
            self.rangerbase.EnableCANCtrl()   

class ScoutBase(GetRobotStae):
    def __init__(self):
        self.basetype='scout'
        self.motronum= 4
        self.scoutbase=agxrobots.UGV()
        self.baseversion=self.scoutbase.base_version
        self.max_linear=1.5
        self.max_angular=0.5235
        self.max_lateral=0.0
        self.min_linear=-self.max_linear
        self.min_angular=-self.max_angular
        self.min_lateral=-self.max_lateral
        if self.baseversion==1:
            self.ScoutActuator1=ActuatorStateMessageV1(motro_id=1)
            self.ScoutActuator2=ActuatorStateMessageV1(motro_id=2)
            self.ScoutActuator3=ActuatorStateMessageV1(motro_id=3)
            self.ScoutActuator4=ActuatorStateMessageV2(motro_id=4)
        elif self.baseversion==2:
            self.ScoutActuator1=ActuatorStateMessageV2(motro_id=1)
            self.ScoutActuator2=ActuatorStateMessageV2(motro_id=2)
            self.ScoutActuator3=ActuatorStateMessageV2(motro_id=3)
            self.ScoutActuator4=ActuatorStateMessageV2(motro_id=4)        

    def  SetMotionCommand(self,linear_vel=0.0,angular_vel=0.0):
        if self.baseversion==2:
            if linear_vel>self.max_linear:
                linear_vel=self.max_linear
            elif linear_vel<self.min_linear:
                linear_vel=self.min_linear
            if angular_vel>self.max_angular:
                angular_vel=self.max_angular
            elif angular_vel<self.min_angular:
                angular_vel=self.min_angular
        self.scoutbase.SetMotionCommand(linear_vel,angular_vel,0,0)
    def SetLightCommand(self,f_mode,f_value,r_mode,r_value):
        self.scoutbase.SetLightCommand(f_mode,f_value,r_mode,r_value)
        # self.version= 

    def EnableCAN(self):
        if self.baseversion==2:
            self.scoutbase.EnableCANCtrl()   
        

class ScoutMiniBase(GetRobotStae):
    def __init__(self):
        self.basetype='scoutMini'
        self.motronum= 4
        self.scoutminibase=agxrobots.UGV()
        self.baseversion=self.scoutminibase.base_version
        self.max_linear=3.0
        self.max_angular=2.5235
        self.max_lateral=2.0
        self.min_linear=-self.max_linear
        self.min_angular=-self.max_angular
        self.min_lateral=-self.max_lateral
        if self.baseversion==1:
            self.ScoutMiniActuator1=ActuatorStateMessageV1(motro_id=1)
            self.ScoutMiniActuator2=ActuatorStateMessageV1(motro_id=2)
            self.ScoutMiniActuator3=ActuatorStateMessageV1(motro_id=3)
            self.ScoutMiniActuator4=ActuatorStateMessageV2(motro_id=4)
        elif self.baseversion==2:
            self.ScoutMiniActuator1=ActuatorStateMessageV2(motro_id=1)
            self.ScoutMiniActuator2=ActuatorStateMessageV2(motro_id=2)
            self.ScoutMiniActuator3=ActuatorStateMessageV2(motro_id=3)
            self.ScoutMiniActuator4=ActuatorStateMessageV2(motro_id=4)

    
    def  SetMotionCommand(self,linear_vel=0.0,angular_vel=0.0,lateral_velocity=0.0):
        if self.baseversion==1:
            if linear_vel<0:
                linear_vel=0
            elif linear_vel>100:
                linear_vel=100
        elif self.baseversion==2:
            if linear_vel>self.max_linear:
                linear_vel=self.max_linear
            elif linear_vel<self.min_linear:
                linear_vel=self.min_linear
            if angular_vel>self.max_angular:
                angular_vel=self.max_angular
            elif angular_vel<self.min_angular:
                angular_vel=self.min_angular
            if lateral_velocity>self.max_lateral:
                lateral_velocity=self.max_lateral
            elif lateral_velocity<self.min_lateral:
                lateral_velocity=self.min_lateral
        self.scoutminibase.SetMotionCommand(linear_vel,angular_vel,lateral_velocity,0)
    def SetLightCommand(self,f_mode,f_value,r_mode,r_value):
        self.scoutminibase.SetLightCommand(f_mode,f_value,r_mode,r_value)
    def EnableCAN(self):
        if self.baseversion==2:
            self.scoutminibase.EnableCANCtrl()
      # self.version= 


class TracerBase(GetRobotStae):
    def __init__(self):
        self.basetype='tracer'
        self.motronum= 2
        self.tracerbase=agxrobots.UGV()
        self.baseversion=self.tracerbase.base_version
        if self.baseversion==1:
            self.TracerActuator1=ActuatorStateMessageV1(motro_id=1)
            self.TracerActuator2=ActuatorStateMessageV1(motro_id=2)
        elif self.baseversion==2:
            self.TracerActuator1=ActuatorStateMessageV2(motro_id=1)
            self.TracerActuator2=ActuatorStateMessageV2(motro_id=2)

    def  SetMotionCommand(self,linear_vel=0.0,angular_vel=0.0):
        self.tracerbase.SetMotionCommand(linear_vel,angular_vel,0,0)
    def SetLightCommand(self,f_mode,f_value):
        self.tracerbase.SetLightCommand(f_mode,f_value)

    def EnableCAN(self):
        if self.baseversion==2:
            self.tracerbase.EnableCANCtrl()
        # self.version= 
    # TracerAuator1=ActuatorStateMessageV2(motro_id=1)
    # TracerAuator2=ActuatorStateMessageV2(motro_id=2)
    