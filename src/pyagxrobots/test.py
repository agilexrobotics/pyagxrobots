#!/usr/bin/env python3
# coding=UTF-8
import pyagxrobots
import time
scoutmini=pyagxrobots.pysdkugv.ScoutMiniBase()
scoutmini.EnableCAN()
num=5
while num>0:
    
    scoutmini.SetMotionCommand(linear_vel=0.1)
    print('rpm')
    print(scoutmini.ScoutMiniActuator2.rpm())
    print('GetLinearVelocity')
    print(scoutmini.GetLinearVelocity())
    time.sleep(0.3)
    num-=1
