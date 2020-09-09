# DJI Python Control GUI
A GUI made to test the DJI SDK control inputs and collecting data in flight experiments. 
From the GUI, you first need to press 'Authorize control', to enable control to be sent to DJI. 
On your remote control, you need the control mode stick to be set to "F" (function) mode. 

The fastest way to abort sending messages is to return the control stick to the "P" mode. 

## Dependencies
This package is dependent on the [Control Authority Service](https://github.com/dji-sdk/Onboard-SDK-ROS/blob/master/srv/SDKControlAuthority.srv), specified in the DJI Onboard SDK for ROS. 
Installation instructions can be found [here](https://github.com/dji-sdk/Onboard-SDK-ROS).


To run the GUI, PyQt5 needs to be installed. This is e.g. done in pip by
```
pip3 install PyQt5
```

The program is also dependent on the ros packages for sensor_msgs to be installed. 
