#!/usr/bin/env python
# coding: utf-8
# Author: Richard Hopkins
# Date: 25 February 2025
#
# This program runs on a
# Pimoroni Motor 2040 
# Quad Motor controller
# running MicroPython
#
# RECEIVE CMD_VEL TWIST FORMAT MESSAGES VIA SERIAL
# 1. Convert the Twist messages into wheel velocities
#   vx = msg.linear.x
#   vy = msg.linear.y
#   wz = msg.angular.z
#   front_left_velocity = (vx - vy - wz) / 3.0
#   front_right_velocity = (vx + vy + wz) / 3.0
#   rear_left_velocity = (vx + vy - wz) / 3.0
#   rear_right_velocity = (vx - vy + wz) / 3.0
# 2. Issue velocity commands to motors
#
# SEND ODOMETER FEEDBACK
# 1. Read time values and encoder values
# 2. Calculate velocity per second
#    Linear Velocity (Vx): Vx = (FR + BR + FL + BL) / 4
#    Angular Velocity (Vθ): `Vθ = (-FR + BR + FL - BL) / (4 * L)
#    L is the distance between the wheels along each axis
# 3. Use the robot's linear and angular velocities to calculate its
#    position over time using the following equations
#       X = X_previous + Vx * cos(θ) * dt
#       Y = Y_previous + Vx * sin(θ) * dt
#       θ = θ_previous + Vθ * dt
# 4. Publish odometry messages containing the robot's position
#    and velocity in the odom frame format via serial line
#
# Serial USB reading by Ben Renninson
# https://github.com/GSGBen/pico-serial
#
# Motor 2040 velocity control by Christopher Parrott
# https://github.com/pimoroni/pimoroni-pico/blob/main/micropython/examples/motor2040/velocity_control.py
#

from sys import stdin
from time import sleep

import uselect
import time
# from machine import Pin

import gc
from motor import Motor, motor2040
from encoder import Encoder, MMME_CPR
from pimoroni import Button, PID, REVERSED_DIR

# Wheel friendly names
FL = 2
FR = 3
RL = 1
RR = 0

GEAR_RATIO = 50                         # The gear ratio of the motors
COUNTS_PER_REV = MMME_CPR * GEAR_RATIO  # The counts per revolution of each motor's output shaft

SPEED_SCALE = 5.4                       # The scaling to apply to each motor's speed to match its real-world speed

UPDATES = 100                           # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
TIME_FOR_EACH_MOVE = 2                  # The time to travel between each value
UPDATES_PER_MOVE = TIME_FOR_EACH_MOVE * UPDATES
PRINT_DIVIDER = 4                       # How many of the updates should be printed (i.e. 2 would be every other update)

DRIVING_SPEED = 1.0                     # The speed to drive the wheels at, from 0.0 to SPEED_SCALE

# PID values
VEL_KP = 30.0                           # Velocity proportional (P) gain
VEL_KI = 0.0                            # Velocity integral (I) gain
VEL_KD = 0.4                            # Velocity derivative (D) gain


# Free up hardware resources ahead of creating a new Encoder
gc.collect()

# Create a list of motors with a given speed scale
MOTOR_PINS = [motor2040.MOTOR_A, motor2040.MOTOR_B, motor2040.MOTOR_C, motor2040.MOTOR_D]
motors = [Motor(pins, speed_scale=SPEED_SCALE) for pins in MOTOR_PINS]

# Create a list of encoders, using PIO 0, with the given counts per revolution
ENCODER_PINS = [motor2040.ENCODER_A, motor2040.ENCODER_B, motor2040.ENCODER_C, motor2040.ENCODER_D]
ENCODER_NAMES = ["RR", "RL", "FL", "FR"]
encoders = [Encoder(0, i, ENCODER_PINS[i], counts_per_rev=COUNTS_PER_REV, count_microsteps=True) for i in range(motor2040.NUM_MOTORS)]

# Reverse the direction of the B and D motors and encoders
motors[FL].direction(REVERSED_DIR)
motors[RL].direction(REVERSED_DIR)
encoders[FL].direction(REVERSED_DIR)
encoders[RL].direction(REVERSED_DIR)

# Create the user button
user_sw = Button(motor2040.USER_SW)

# Create PID objects for position control
vel_pids = [PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE) for i in range(motor2040.NUM_MOTORS)]

# Helper functions for driving in common directions
def drive_forward(speed):
    vel_pids[FL].setpoint = speed
    vel_pids[FR].setpoint = speed
    vel_pids[RL].setpoint = speed
    vel_pids[RR].setpoint = speed


def turn_right(speed):
    vel_pids[FL].setpoint = speed
    vel_pids[FR].setpoint = -speed
    vel_pids[RL].setpoint = speed
    vel_pids[RR].setpoint = -speed


def strafe_right(speed):
    vel_pids[FL].setpoint = speed
    vel_pids[FR].setpoint = -speed
    vel_pids[RL].setpoint = -speed
    vel_pids[RR].setpoint = speed


def stop():
    vel_pids[FL].setpoint = 0
    vel_pids[FR].setpoint = 0
    vel_pids[RL].setpoint = 0
    vel_pids[RR].setpoint = 0

# Enable the motor to get started
for m in motors:
    m.enable()

update = 0
print_count = 0
sequence = 0

captures = [None] * motor2040.NUM_MOTORS

buffered_input = []
input_line_this_tick = ""
TERMINATOR = "\n"
latest_input_line = ""

# Continually move the motor until the user button is pressed
while not user_sw.raw():

    # Capture the state of all the encoders
    for i in range(motor2040.NUM_MOTORS):
        captures[i] = encoders[i].capture()

    for i in range(motor2040.NUM_MOTORS):
        # Calculate the acceleration to apply to the motor to move it closer to the velocity setpoint
        accel = vel_pids[i].calculate(captures[i].revolutions_per_second)

        # Accelerate or decelerate the motor
        motors[i].speed(motors[i].speed() + (accel * UPDATE_RATE))

    # Print out the current motor values, but only on every multiple
    if print_count == 0:
        for i in range(motor2040.NUM_MOTORS):
            print(ENCODER_NAMES[i], "=", captures[i].revolutions_per_second, end=", ")
        print()

    # Increment the print count, and wrap it
    print_count = (print_count + 1) % PRINT_DIVIDER

    update += 1     # Move along in time

    # Have we reached the end of this movement?
    if update >= UPDATES_PER_MOVE:
        update = 0  # Reset the counter

        # Move on to the next part of the sequence
        sequence += 1

        # Loop the sequence back around
        if sequence >= 7:
            sequence = 0

    # Set the motor speeds, based on the sequence
    if sequence == 0:
        drive_forward(DRIVING_SPEED)
    elif sequence == 1:
        drive_forward(-DRIVING_SPEED)
    elif sequence == 2:
        turn_right(DRIVING_SPEED)
    elif sequence == 3:
        turn_right(-DRIVING_SPEED)
    elif sequence == 4:
        strafe_right(DRIVING_SPEED)
    elif sequence == 5:
        strafe_right(-DRIVING_SPEED)
    elif sequence == 6:
        stop()

    time.sleep(UPDATE_RATE)

# Stop all the motors
for m in motors:
    m.disable()

def read_serial_input():
    global buffered_input, input_line_this_tick, TERMINATOR
    select_result = uselect.select([stdin], [], [], 0)
    while select_result[0]:
        input_character = stdin.read(1)
        buffered_input.append(input_character)
        select_result = uselect.select([stdin], [], [], 0)
    if TERMINATOR in buffered_input:
        line_ending_index = buffered_input.index(TERMINATOR)
        input_line_this_tick = "".join(buffered_input[:line_ending_index])
        if line_ending_index < len(buffered_input):
            buffered_input = buffered_input[line_ending_index + 1 :]
        else:
            buffered_input = []
    else:
        input_line_this_tick = ""


# main loop
def main():
    '''
    Main loop
    '''
    while True:
        # check for instructions from Pi
        read_serial_input()
        if input_line_this_tick:
            latest_input_line = input_line_this_tick
            command = ""
            command = latest_input_line.strip()
            if command == "odm":
                dt = int(time.time() * 1000) - time_ms
                time_ms = time_ms + dt
                # get encoder difference
                #
                # Vx = (FR + BR + FL + BL) / 4
                # Vθ = (-FR + BR + FL - BL) / (4 * L)
                # X = X_previous + Vx * cos(θ) * dt
                # Y = Y_previous + Vx * sin(θ) * dt
                # θ = θ_previous + Vθ * dt
                #
                # write odm data back to the serial line

            if command == "cmd_vel":
                # write velocities to motors
                pass

if __name__ == '__main__':
    main()