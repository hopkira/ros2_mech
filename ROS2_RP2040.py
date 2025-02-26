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
#  Update the package dependencies in `package.xml`:
# ```xml
# <depend>rclpy</depend>
# <depend>geometry_msgs</depend>
#
# Add an entry point for your node in `setup.py`. Replace the existing 
# executable declaration with this:
# ```python
# entry_points={
#        'console_scripts': [
#                'my_mechanum_wheels = 
#my_mechanum_wheels.RP2040_mechanum_controller_node:main'
#        ],
#},
#
# Build the package and source your ROS environment:
# ```bash
# cd ~/ros2_ws
# colcon build --packages-select my_mechanum_wheels
# source install/setup.bash
#
# Run the node to subscribe to `cmd_vel` topic:
#```bash
#ros2 run my_mechanum_wheels my_mechanum_wheels
#
# REACT TO MESSAGES
# a. Subscribe to /cmd_vel Twist messages
# b. Turn these into serial messages
#
# RECEIVE ODOMETER FRAMES FROM SERIAL
# a. Publish odometer frames


import serial
import rclpy
import ast
import time
from geometry_msgs.msg import Twist


class MotorController(rclpy.Node):
    def __init__(self) -> None:
        super().__init__('RP2040_mechanum_controller_node')
        self.subscription = self.create_subscription(Twist, 
            '/cmd_vel', self.listener_callback, 10)
        self.ser = serial.Serial(
            port='/dev/backpanel', # replace with your device name
            baudrate = 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
            )
        self.time_ms = int(time.time() * 1000)

    def listener_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        front_left_velocity = (vx - vy - wz) / 3.0
        front_right_velocity = (vx + vy + wz) / 3.0
        rear_left_velocity = (vx + vy - wz) / 3.0
        rear_right_velocity = (vx - vy + wz) / 3.0
        # send four velocities to wheels
        # odm,0,0,0,0
        # self.__write(str.encode(twist+"\n"))

    def __write(self,text:str) -> None:
        #print("Back panel:",text)
        self.ser.write(str.encode(text+"\n"))

    def cmd(self,text:str) -> None:
        self.__write(text)

    def on(self):
        self.__write("original")

    def off(self):
        self.__write("off")

    def send_odm(self) -> list:
        odm = []
        self.__write("odm")
        my_input = self.ser.readlines()
        #print("I heard:" + str(my_input))
        try:
            input_str = my_input[0].decode().strip()
        except IndexError:
            return odm
        # read odm message
        return odm

def main(args=None):
    rclpy.init(args=args)
    RP2040_mechanum_controller_node = MotorController()
    rclpy.spin(RP2040_mechanum_controller_node)
    RP2040_mechanum_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()