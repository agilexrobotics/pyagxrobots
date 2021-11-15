# Example Package

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

You can run 1-5 for the first-time setup and run 2 to bring up the device each time you  unplug and re-plug the adapter.

