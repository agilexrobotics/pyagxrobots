# This is Python API for Agilex Robotics

###### This is a python API for CAN communication with Agilex Robotics

### Installation

##### Notes:

###### Make sure that python version >=3.4. if you have python3 and python2 ,please use python3

**Communication protocol**

|   Robot    | Protocol V2 | CAN  | Support Status |
| :--------: | :---------: | :--: | :------------: |
| Scout 2.0  |      Y      |  Y   |     Active     |
| Scout Mini |      Y      |  Y   |     Active     |
| Hunter 2.0 |      Y      |  Y   |     Active     |
|   Bunker   |      Y      |  Y   |     Active     |
|   Tracer   |      Y      |  Y   |     Active     |

#### pip

````bash
$ pip3 install pyagxrobots
````

##### Notes:

###### Make sure that pip3 version >= 9.0.0. 

###### cheak the pip3 version

```bash
$ pip3 -V     		
```

###### updata pip3

```bash
$ python3 -m pip install --upgrade pip                 
```

### Setup CAN-To-USB adapter

1. Enable gs_usb kernel module

   ```bash
   $ sudo modprobe gs_usb
   ```

2. Bringup can device

   ```bash
   $ sudo ip link set can0 up type can bitrate 500000
   ```

3. If no error occured during the previous steps, you should be able to see the can device now by using command

   ```bash
   $ ifconfig -a
   ```

4. Install and use can-utils to test the hardware

   ```bash
   $ sudo apt install can-utils
   ```

5. Testing command

   ```bash
   # receiving data from can0
   $ candump can0
   # send data to can0
   $ cansend can0 001#1122334455667788
   ```

You can run  step 1-5 or "./setup_can2usb.bash"  for the first-time setup and run step 2 or"./bringup_can2usb.bash"  to bring up the device each time you  unplug and re-plug the adapter.

### import to your project

```python
#!/usr/bin/env python3
# coding=utf-8
import pyagxrobots
robots=pyagxrobots.pysdkugv.robotstype()  #    robotstype depend on your robot
```

#### function list:

```python
EnableCAN()
SetMotionCommand()
SetLightCommand()
GetRobotStae:
    MotionCommandMessage:
        GetLinearVelocity()
        GetAngularVelocity()
        GetLateralVelocity()
        GetSteeringAngle()
    LightCommandMessage:
        GetLightCmdCtrl()
        GetFrontMode()
        GetFrontCustom()
        GetRearMode()
        GetRearCustom()
    SystemStateMessage:
        GetVehicleState()
        GetControlMode()
        GetBatteryVoltage()
        GetErrorCode()
    RcStateMessage:
    OdometryMessage:
        GetLeftWheel()
        GetRightWheel()
    ActuatorStateMessageV2:
        rpm()
        current()
        pulse_count()
        driver_voltage()
        driver_temp()
        motor_temp()
        driver_state()
```

#### EnableCAN

- **Prototype**: `EnableCAN()`
- **Description**: Enable command and control mode.

#### SetMotionCommand

- **Prototype**: `SetMotionCommand()`
- **Description**:Send Version Request to robots.
- **Parameters**
  - `linear_vel`:(float) 
  - `angular_vel`:(float)
  - `lateral_velocity`:(float)
  - `steering_angle`:(float)

#### GetLinearVelocity

- **Prototype**: `GetLinearVelocity()`
- **Description**:Get the linear velocity from robot
- **Return**:linear velocity

#### GetAngularVelocity

- **Prototype**: `GetAngularVelocity()`
- **Description**:Get the angular velocity from robot.
- **Return**:angular velocity

#### GetSteeringAngle

- **Prototype**: `GetSteeringAngle()`
- **Description**:Get the steering angle from robot .
- **Return**:steering angle

#### GetLateralVelocity

- **Prototype**: `GetLateralVelocity()`
- **Description**:Get the lateral velocity from robot .
- **Return**:lateral velocity

#### GetControlMode

- **Prototype**: `GetControlMode()`
- **Description**:Get the control mode from robot .
- **Return**:control mode

#### GetLeftWheelOdeom

- **Prototype**: `GetLeftWheelOdeo()`
- **Description**:get robots LeftWheelOdom .
- **Return**:LeftWheelOdom

#### GetRightWheelOdom

- **Prototype**: `GetRightWheelOdom()`
- **Description**:get robots RightWheelOdom .
- **Return**:RightWheelOdeom

#### GetBatteryVoltage

- **Prototype**: `GetBatteryVoltage()`
- **Description**:Get the battery voltage from robot
- **Return**:battery voltage

#### GetErrorCode

- **Prototype**: `GetErrorCode()`
- **Description**:Get the error code from robot
- **Return**:error code

### Example

#### Note:

##### For safety, please ensure that the robot's wheels are off the ground

#### 1.ScoutMini

```python
#!/usr/bin/env python3
# coding=UTF-8
import pyagxrobots
import time
scoutmini=pyagxrobots.pysdkugv.ScoutMiniBase()
scoutmini.EnableCAN()
num=5
while num>0:
    
    scoutmini.SetMotionCommand(linear_vel=0.1)
    print(scoutmini.GetLinearVelocity())
    time.sleep(0.3)
    num-=1
```

#### 2.Bunker

```python
#!/usr/bin/env python3
# coding=UTF-8
import pyagxrobots
import time
bunker=pyagxrobots.pysdkugv.BunkerBase()
bunker.EnableCAN()
num=5
while num>0:
    
    bunker.SetMotionCommand(linear_vel=0.1)
    print(bunker.GetLinearVelocity())
    time.sleep(0.3)
    num-=1
```
