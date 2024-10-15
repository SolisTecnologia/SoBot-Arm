# Solis Robot - SoBot
![](https://github.com/SolisTecnologia/SoBot-Arm/blob/master/png/SoBotArm.png)
# Introduction

AMR (autonomous mobile robotics) platform equipped with a camera system, ultrasonic and photoelectric sensors, works with a high rate of precision and repeatability of its movements, as it uses stepper motors in its movement and navigation, the SoBot also can be termed as a research and development interface, as it facilitates the practical experimentation of algorithms from the simplest to the most complex level.

This product was developed 100% by Solis Tecnologia, and has a lot of technology employing cutting-edge concepts, such as:

The motors can be controlled simultaneously or individually.
The user can select different accessories to implement to the robot.
Several programming languages can be used to connect via API.

# Components

* Main structure in aluminum
* Robot Control Driver
* Raspberry Pi 4B board <img align="center" height="30" width="40" src="https://github.com/devicons/devicon/blob/master/icons/raspberrypi/raspberrypi-original.svg">
* 2x NEMA-23 Stepper Motors
* 2x 12V/5A battery
* Robotic Arm - Dobot Magician Lite

# Programming Example
## SOBOT ARM - [SOBOT_ARM.py](https://github.com/SolisTecnologia/SoBot-Arm/blob/master/DOBOT/SOBOT_ARM.py)

This program was developed to demonstrate how the integration of the control of the robotic arm coordinated with the movement of the SoBot can be done to perform automated object manipulation tasks.

___

### Programming Language

* Python  <img align="center" height="30" width="40" src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg">

___


### Packages required for installation

Procedure for installing the packages required for the Dobot Magician Lite robotic arm library to work.  

```
•sudo date MMDDHHMMYYYY
    The data are:
    MM = Month
    DD = Day
    HH = Hour
    MM = Minute
    YYYY = Year (with 4 digits)

•sudo apt-get update

•sudo apt-get upgrade -y

•sudo apt-get install -y libqt5serialport5 libqt5serialport5-dev

```
The “sudo date” command updates the Raspberry time so that the update and upgrade commands can be executed correctly.  
After executing this sequence of commands, it is now possible to use the Dobot library (DobotDllType) to control the robotic arm.  
All the libraries required for operation are included in the control example file.  
___

### Required Libraries

~~~python
import serial
import serial.tools.list_ports
from time import sleep
import threading
import DobotDllType as dType
~~~

* **Serial:** For serial communication between the SoBot and the Raspberry.  
* **Threading:** Library to perform operations in parallel.  
* **DobotDllType:** Library for controlling Dobot's robotic arm.  
___

### Code Description

The program executes movements according to the defined states to move the SoBot and the robotic arm in synchrony.
___

* #### MAIN FUNCTION


___

* #### AUXILIARY FUNCTIONS

*1. Serial Device Finder*
~~~python
"""
Function to find the serial port that the SoBot board is connected to
"""
def serial_device_finder (name_device):
~~~

This function finds the serial port that the SoBot device is connected to by comparing the device name with the descriptions of the ports available in the system. 
___

*2. Read Seria USB*
~~~python
"""
Function to read commands received via USB serial port from SoBot
"""
def Read_Serial():
~~~

The Read_SerialUSB function is responsible for reading data received from the serial port sent by SoBot. 



___

For more information about the commands used, check the Robot Commands Reference Guide.
  
# Reference Link
[SolisTecnologia website](https://solistecnologia.com/produtos/robotsingle)

# Please Contact Us
If you have any problem when using our robot after checking this tutorial, please contact us.

### Phone:
+55 1143040786

### Technical support email: 
contato@solistecnologia.com.br

![](https://github.com/SolisTecnologia/SoBot-USB-Control/blob/master/png/logo.png)