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
• sudo date MMDDHHMMYYYY
    The data are:
    MM = Month
    DD = Day
    HH = Hour
    MM = Minute
    YYYY = Year (with 4 digits)

• sudo apt-get update

• sudo apt-get upgrade -y

• sudo apt-get install -y libqt5serialport5 libqt5serialport5-dev

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

For more information about the robotic arm, please see the following links:  
[DOWNLOAD CENTER – DOBOT MAGICIAN LITE](https://www.dobot-robots.com/service/download-center?keyword=&products%5B%5D=315)  
[DOWNLOAD CENTER – DOBOT MAGICIAN](https://www.dobot-robots.com/service/download-center?keyword=&products%5B%5D=316)  
[Dobot Magician with Python – Starter Guide](https://github.com/SERLatBTH/StarterGuide-Dobot-Magician-with-Python)
___

### Code Description

The program executes movements according to the defined states to move the SoBot and the robotic arm in synchrony.
___

* #### MAIN FUNCTION

1. The algorithm identifies and connects to serial devices.  
~~~python
# Serial connection of the SoBot power and control board
usb_SoBot = serial.Serial()
serial_SoBot = serial_device_finder(Desc_SoBot_serial)

# Serial connection to the Magician Lite Arm
usb_Dobot = [0]
api = dType.load()
serial_Dobot = serial_device_finder(Desc_Dobot_serial)
~~~

2. Executes thread to monitor commands received in real time.  
~~~python
# Thread to monitor the Sobot USB serial port
T_read_serial = threading.Thread(target=Read_Serial, daemon=True)
T_read_serial.start()
~~~

3. Configures the SoBot to send feedback commands for its movement.  
~~~python
# SoBot Initial Settings
usb_SoBot.write(b"CR1")
usb_SoBot.write(b"MT0 E1")
usb_SoBot.write(b"LT CR0")
usb_SoBot.write(b"BZ CR0")
usb_SoBot.write(b"LT E1 RD0 GR0 BL100")
~~~

4. Sends configuration and control commands to the robotic arm.  
~~~python
dType.SetQueuedCmdClear(api)

MagicianIndexGet = dType.SetEndEffectorSuctionCup(api, False, True, isQueued=0)     # Disable Suction Command

#Configurações de Movimento
dType.SetHOMEParams(api, 240, 0, 150, 0, isQueued = 1)                # Sets the default position of the Dobot Magicia
dType.SetPTPJointParams(api, 80, 80, 80, 80, 80, 80, 80, 80, isQueued = 1)  # Sets the joint parameters
dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)                       # Defines the velocity ratio and acceleration ratio in PTP mode
dType.SetPTPCoordinateParams(api, 200, 200, 400, 200, isQueued = 1)         # Set the velocity and acceleration of the Cartesian coordinate axis en PTP mode
                                                                            # (api, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued)

dType.SetHOMECmd(api, temp = 0, isQueued = 1)       # Command to go to home position

MagicianIndex = dType.SetPTPCmd(api, 2, 225, 180, 20, 0, isQueued = 1)[0]   # PTP mode = 2 - Set straight line path between two points
                                                                            # Coordinate parameters in PTP mode (x,y,z,r)
                                                                            # set to Cartesian coordinate
~~~

5. Within a looping programming, it uses the state machine concept to manage the flow of tasks by moving the SoBot and robotic arm in an orderly and synchronized manner until all tasks are completed.  


___

* #### AUXILIARY FUNCTIONS

1. Serial Device Finder*
~~~python
"""
Function to find the serial port that the SoBot board is connected to
"""
def serial_device_finder (name_device):
~~~

This function finds the serial port that the SoBot device is connected to by comparing the device name with the descriptions of the ports available in the system. 
___

2. Read Seria USB*
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
[SolisTecnologia website](https://www.solistecnologia.com.br/produtos/estacoes_sobot)

# Please Contact Us
If you have any problem when using our robot after checking this tutorial, please contact us.

### Phone:
+55 1143040786

### Technical support email: 
contato@solistecnologia.com.br

![](https://github.com/SolisTecnologia/SoBot-USB-Control/blob/master/png/logo.png)