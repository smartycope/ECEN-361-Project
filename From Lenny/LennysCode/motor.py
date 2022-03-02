#!/usr/bin/python

""" This just holds a few convenience functions for moving the motor using the RRB3 library """

from rrb3 import *
import time

raw = RRB3(11, 11)

def constrain(speed):
    if speed > 1:
        speed = 1
    if speed < 0:
        speed = 0
    return speed

def directionTest(key):
    if (key == 0): # UP
        print('Up!!!')
    
    elif (key == 1): # DOWN
        print('Down!!!')
        
    elif (key == 2): # LEFT
        print('Left!!!')
        
    elif (key == 3): # RIGHT
        print('Right!!!')
            

# These may be swapped, I haven't tested it
def forward(speed):
    raw.set_motors(constrain(speed), 0, constrain(speed), 0)

def reverse(speed):
    raw.set_motors(constrain(speed), 1, constrain(speed), 1)

# These may be swapped, I haven't tested it
def left(speed):
    raw.set_motors(constrain(speed), 1, constrain(speed), 0)

def right(speed):
    raw.set_motors(constrain(speed), 0, constrain(speed), 1)

def stop():
    raw.set_motors(0, 0, 0, 0)
