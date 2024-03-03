# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Simple demo of reading and writing the digital I/O of the MCP2300xx as if
# they were native CircuitPython digital inputs/outputs.
# Author: Tony DiCola
import time

import board
import busio
import digitalio

#from adafruit_mcp230xx.mcp23008 import MCP23008

from adafruit_mcp230xx.mcp23017 import MCP23017


# Initialize the I2C bus:
i2c = busio.I2C(board.SCL, board.SDA)

# Create an instance of either the MCP23008 or MCP23017 class depending on
# which chip you're using:
#mcp = MCP23008(i2c)  # MCP23008
mcp = MCP23017(i2c, address=0x20)  # MCP23017

# Optionally change the address of the device if you set any of the A0, A1, A2
# pins.  Specify the new address with a keyword parameter:
# mcp = MCP23017(i2c, address=0x21)  # MCP23017 w/ A0 set

# Now call the get_pin function to get an instance of a pin on the chip.
# This instance will act just like a digitalio.DigitalInOut class instance
# and has all the same properties and methods (except you can't set pull-down
# resistors, only pull-up!).  For the MCP23008 you specify a pin number from 0
# to 7 for the GP0...GP7 pins.  For the MCP23017 you specify a pin number from
# 0 to 15 for the GPIOA0...GPIOA7, GPIOB0...GPIOB7 pins (i.e. pin 12 is GPIOB4).
a = mcp.get_pin(8)
#pin1 = mcp.get_pin(9)

# Setup pin0 as an output that's at a high logic level.
a.switch_to_input(value=True)
b = mcp.get_pin(9)
b.switch_to_input(value=True)
#pin1.switch_to_output(value=True)

# Setup pin1 as an input with a pull-up resistor enabled.  Notice you can also
# use properties to change this state.
#pin1.direction = digitalio.Direction.INPUT
#pin1.pull = digitalio.Pull.UP

# Now loop blinking the pin 0 output and reading the state of pin 1 input.
"""
   current_state_A = GPIO.input(self.ch_a)
    current_state_B = GPIO.input(self.ch_b)
    if current_state_A != self.last_state_A:
        if current_state_B != current_state_A:  
            self.pulse_count += 1  # Clockwise (increment) 

        else:
            self.pulse_count -= 1  # Counter-clockwise (decrement)
            """
currentStateA = a.value
currentStateB = b.value
pulse_count = 0
while True:
    if currentStateA != a.value:
        if currentStateB != currentStateA:
            print("Clockwise", pulse_count)
            pulse_count += 1
        else:
            print("Counter-clockwise", pulse_count)
            pulse_count -= 1
        currentStateA = a.value
        currentStateB = b.value