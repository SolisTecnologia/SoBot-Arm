#!/usr/bin/python3
"""
Solis Robot - SoBot

SOBOT_ARM.py: Programming example where the SoBot moves 
to manipulate an object with the help of the robotic arm.

Created By   : Vinicius M. Kawakami
Version      : 1.0

Company: Solis Tecnologia

"""

import serial
import serial.tools.list_ports
from time import sleep
import threading
import DobotDllType as dType

'''
###################################
        Global Variables
###################################
'''

MagicianIndex = 0
state = 0

# USB Serial Port Descriptions
Desc_SoBot_serial = "VCOM"
Desc_Dobot_serial = "Virtual COM Port - Hiker sudio"

# Settings Magician Lite
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

'''
###################################
        Auxiliary Functions
###################################
'''

"""
Function to find the serial port that the SoBot board is connected to
"""
def serial_device_finder (name_device):
    # List all available serial ports
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # Check if the port description contains the device name
        if name_device in port.description:
            print(f"Device found: {port.device} - {port.description}")
            return port.device

    # If can't find the device
    print(f"Device '{name_device}' not found.")
    return None

"""
Function to read commands received via USB serial port from SoBot
"""
def Read_Serial():
    global state
    Index = 0

    while True:
        command = usb_SoBot.readline() # Read data
        if(command != b''):
            print("Command received",command) 
            command = command[0:9]
            if(command == b'CR OK MT0'):
                Index += 1
                print("Index =",Index, "State:", state)
                if(Index == 2):
                    state = 2
                elif(Index == 3):
                    state = 5

'''
###################################
        Main Function
###################################
'''
# Serial connection of the SoBot power and control board
usb_SoBot = serial.Serial()
serial_SoBot = serial_device_finder(Desc_SoBot_serial)

if serial_SoBot :
    # Connect to the found device
    usb_SoBot = serial.Serial(serial_SoBot, baudrate=57600, timeout=0, dsrdtr=False)
    usb_SoBot.flush()
    print(f"Connected to device: {serial_SoBot}")
else:
    print("No device 0 was connected.")

# Serial connection to the Magician Lite Arm
usb_Dobot = [0]
api = dType.load()
serial_Dobot = serial_device_finder(Desc_Dobot_serial)

if serial_Dobot:
    # Connect to the found device
    usb_Dobot = dType.ConnectDobot(api, serial_Dobot, 115200)[0]

    print("Connect Dobot status:",CON_STR[usb_Dobot])
else:
    print("No device 1 was connected.")

# Thread to monitor the Sobot USB serial port
T_read_serial = threading.Thread(target=Read_Serial, daemon=True)
T_read_serial.start()

if (usb_Dobot == dType.DobotConnect.DobotConnect_NoError):
    # SoBot Initial Settings
    usb_SoBot.write(b"CR1")
    usb_SoBot.write(b"MT0 E1")
    usb_SoBot.write(b"LT CR0")
    usb_SoBot.write(b"BZ CR0")
    usb_SoBot.write(b"LT E1 RD0 GR0 BL100")

    dType.SetQueuedCmdClear(api)

    MagicianIndexGet = dType.SetEndEffectorSuctionCup(api, False, True, isQueued=0)     # Disable Suction Command

    #Configurações de Movimento
    dType.SetHOMEParams(api, 240, 0, 150, 0, isQueued = 1)                # Sets the default position of the Dobot Magicia
    dType.SetPTPJointParams(api, 80, 80, 80, 80, 80, 80, 80, 80, isQueued = 1)  # Sets the joint parameters
    dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)                       # Defines the velocity ratio and acceleration ratio in PTP mode
    dType.SetPTPCoordinateParams(api, 200, 200, 400, 200, isQueued = 1)         # Set the velocity and acceleration of the Cartesian coordinate axis en PTP mode
                                                                                # (api, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued)

    dType.SetHOMECmd(api, temp = 0, isQueued = 1)       # Command to go to home position

    MagicianIndex = dType.SetPTPCmd(api, 2, 225, 180, 20, 0, isQueued = 1)[0]     # PTP mode = 2 - Set straight line path between two points
                                                                                                    # Coordinate parameters in PTP mode (x,y,z,r)
                                                                                                    # set to Cartesian coordinate
    MagicianIndex = dType.SetPTPCmd(api, 2, 225, -180, 20, 0, isQueued = 1)[0]
    MagicianIndex = dType.SetPTPCmd(api, 2, 240, 0, 150, 0, isQueued = 1)[0]

    while MagicianIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        sleep(0.25)

    move = 1
    while(move):

        if(state == 0):
            # Move the SoBot forward
            usb_SoBot.write(b"LT E1 RD0 GR100 BL0")
            usb_SoBot.write(b"MT0 D500 AT500 DT500 V6")
            state = 1

        elif(state == 2):
            usb_SoBot.write(b"LT E1 RD100 GR5 BL0")
            # Simulates arm movement using suction to move an object
                                            #     (   x,    y,   z,  r)
            MagicianIndex = dType.SetPTPCmd(api, 2, 250,  100, 150, 0, isQueued = 1)[0]
            MagicianIndexGet = dType.SetEndEffectorSuctionCup(api, True, True, isQueued=1)      # Enable Suction Command
            MagicianIndex = dType.SetPTPCmd(api, 2, 260,  100,  80, 0, isQueued = 1)[0]
            # Collect the object
            MagicianIndex = dType.SetPTPCmd(api, 2, 240,    0, 150, 0, isQueued = 1)[0]
            MagicianIndex = dType.SetPTPCmd(api, 2, 250, -100, 150, 0, isQueued = 1)[0]
            MagicianIndex = dType.SetPTPCmd(api, 2, 260, -100,  80, 0, isQueued = 1)[0]
            MagicianIndexGet = dType.SetEndEffectorSuctionCup(api, False, True, isQueued=1)     # Disable Suction Command
            # Place the object
            MagicianIndex = dType.SetPTPCmd(api, 2, 260, -100, 120, 0, isQueued = 1)[0]
            MagicianIndex = dType.SetPTPCmd(api, 2, 240,    0, 150, 0, isQueued = 1)[0]

            # Wait for the commands sent to the arm to finish
            while MagicianIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                sleep(0.25)
            
            state = 3

        elif(state == 3):
            # Move the SoBot forward
            usb_SoBot.write(b"LT E1 RD0 GR100 BL0")
            usb_SoBot.write(b"MT0 D-500 AT500 DT500 V6")
            state = 4

        elif(state == 5):
            move = 0


#Disconnect Dobot
dType.DisconnectDobot(api)
usb_SoBot.write(b"MT0 E0")
usb_SoBot.write(b"LT E0")

