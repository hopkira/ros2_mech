#!/usr/bin/env python
# coding: utf-8
# Author: Richard Hopkins
# Date: 25 February 2025
#
# This ROS2 service interfaces to the
# Pimoroni RP2040 Quad Motor Controller
# RP2040_Motor_Controller.py
# 
# The program interacts with the panel.py
# program.  The panel.py program can be uploaded
# to the micropython device using:
#
# python3 pyboard.py --device /dev/tty.usbmodem387A384631342 -f cp panel.py :main.py
#
#

import serial
import ast

class MotorController():
    def __init__(self) -> None:
        self.ser = serial.Serial(
            port='/dev/backpanel', # replace with your device name
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
            )

    def __write(self,text:str) -> None:
        #print("Back panel:",text)
        self.ser.write(str.encode(text+"\n"))

    def cmd(self,text:str) -> None:
        self.__write(text)

    def on(self):
        self.__write("original")

    def off(self):
        self.__write("off")
