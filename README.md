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
* USB control  <img align="center" height="40" width="40" src="https://github.com/SolisTecnologia/SoBot-USB-Control/blob/master/png/control.png">

# Programming Example
## Learning Mode - [LearningMode.py](https://github.com/SolisTecnologia/SoBot-Learning-Mode/blob/master/LearningMode.py)

For this programming, the SoBot was equipped with a camera and a Logitech F710 wireless controller. The robot is capable of recording and executing commands based on inputs received from the controller, allowing the movements to be replicated later. In addition, the SoBot's camera provides a real-time view of the path taken, which facilitates navigation and supervision of the robot in real time.


___

### Programming Language

* Python  <img align="center" height="30" width="40" src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg">

___

### Required Libraries

~~~python
import multiprocessing
import evdev
from evdev import InputDevice, categorize, ecodes
from time import sleep, perf_counter
import serial
import serial.tools.list_ports
import cv2
import numpy as np
~~~

* **Multiprocessing:** To handle simultaneous processes, such as reading devices and executing commands.  
* **Evdev:** To capture events generated by the Logitech controller.  
* **Serial:** For serial communication between the SoBot and the Raspberry.  
* **Cv2 (OpenCV):** For capturing and processing video from the robot's integrated camera.  
* **Numpy:** For manipulating arrays.  
___

### Code Description


___

* #### MAIN FUNCTION


___

* #### AUXILIARY FUNCTIONS


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