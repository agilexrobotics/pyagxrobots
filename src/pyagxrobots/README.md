# This is Python API for pyagxrobots

###### This is a python API for Can communication with pyagxrobots and controlling it.

### Installation

##### Notes:

###### Make sure that python version >=3.6. if you have python3 and python2 ,please use python3

#### pip

````bash
pip3 install pyagxrobots
````

##### Notes:

###### Make sure that pip3 version >= 9.0.0. 

```bash
pip3 -V     																				 #cheak the pip3 version
python3 -m pip install --upgrade pip                          #updata pip3
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

You can run  1-5 for the first-time setup and run 2 to bring up the device each time you  unplug and re-plug the adapter.

### import to your project

```python
#!/usr/bin/env python3
# coding=utf-8
from pyagxrobots import agxrobots
robots=agxrobots.UGV(bustype='socketcan', channel='can0', bitrate=500000) 
```

#### function list:

```python
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
```

#### EnableCANCtrl

- **Prototype**: `EnableCANCtrl()`
- **Description**:Enable Controller Area Network control.

#### SendVersionRequest

- **Prototype**: `SendVersionRequest()`
- **Description**:Send Version Request to robots.

#### SendErrorClearByte

- **Prototype**: `SendErrorClearByte(id)`
- **Description**:Send Error Clear Byte to robots.
- **Parameters**
  - `id`:(int) 0-4    0:all  1~4 :clear motro 1~4

#### EnableLightCtrl

- **Prototype**: `EnableLightCtrl()`
- **Description**:Enable Light control to robots.

#### DisableLightCtrl

- **Prototype**: `DisableLightCtrl()`
- **Description**:Enable Light control to robots.

#### LightFrontMode

- **Prototype**: `LightFrontMode(mode,bright)`
- **Description**:Send Error Clear Byte to robots.
- **Parameters**
  - `mode`:(int)0~3   0:often shut 1:normally open 2:breathing lamp  3:custom
  - `bright`:(int) 0~100  Note: mode must be 3

#### SendLinerVelocity

- **Prototype**: `SendLinerVelocity(liner_velocity)`
- **Description**:Send liner_velocity to robots.
- **Parameters**
  - `liner_velocity`:(float)-3.0~3.0 m/s

#### SendAngularVelocity

- **Prototype**: `SendAngularVelocity(angular_velocity)`
- **Description**:Send angular_velocity to robots.
- **Parameters**
  - `angular_velocity`:(float)-2.523~2.523  rad/s

#### GetLightMode

- **Prototype**: `GetLightMode()`
- **Description**:get robots light mode .
- **Return**:light mode

#### GetSysVersion

- **Prototype**: `GetSysVersion()`
- **Description**:get robots system version .
- **Return**:ControlHardwareVersion,ActuarorHardwareVersion,ControlSoftwareVersion,GetActuarorSoftwareVersion

#### GetLeftWheelOdem

- **Prototype**: `GetLeftWheelOdem()`
- **Description**:get robots LeftWheelOdem .
- **Return**:LeftWheelOdem

#### GetRightWheelOdem

- **Prototype**: `GetRightWheelOdem()`
- **Description**:get robots RightWheelOdem .
- **Return**:RightWheelOdem

#### GetLinerVelocity

- **Prototype**: `GetLinerVelocity()`
- **Description**:get robots liner_velocity .
- **Return**:liner_velocity

#### GetAngularVelocity

- **Prototype**: `GetAngularVelocity()`
- **Description**:get robots angular_velocity .
- **Return**:angular_velocity

#### GetErrorCode

- **Prototype**: `GetErrorCode()`
- **Description**:get robots error_code  .
- **Return**:error_code

### Example

#### Note:

##### For safety, please ensure that the robot's wheels are off the ground

#### 1.Open  Front Light

```python
#!/usr/bin/env python3
# coding=utf-8
from pyagxrobots import agxrobots
robots=agxrobots.UGV(bustype='socketcan', channel='can0', bitrate=500000) 
robots.EnableCANCtrl()
robots.EnableLightCtrl()
robots.LightFrontMode(1)
```

#### 2.Move Robot

```python
#!/usr/bin/env python3
# coding=utf-8
from pyagxrobots import agxrobots
robots=agxrobots.UGV(bustype='socketcan', channel='can0', bitrate=500000) 
robots.EnableCANCtrl()
robots.SendLinerVelocity(0.2)
```

#### 3.get CAN message

```python
#!/usr/bin/env python3
# coding=utf-8
from pyagxrobots import agxrobots
robots=agxrobots.UGV(bustype='socketcan', channel='can0', bitrate=500000) 
robots.EnableCANCtrl()
robots.GetAngularVelocity()
robots.GetLinerVelocity()
```
