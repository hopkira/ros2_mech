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
from sys import stdin
from time import sleep

import uselect
from machine import Pin

# Serial USB reading by Ben Renninson
# https://github.com/GSGBen/pico-serial
buffered_input = []
input_line_this_tick = ""
TERMINATOR = "\n"
latest_input_line = ""

patterns = {
    "left": [],
    "right": [],
    "forward": [],
    "back": [],
    "spin_l": [],
    "spin_r" : []
}

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
            # if the command is a light
            # sequence then switch to that
            # one and reset phase
            if command in patterns:
                pattern = command

main()