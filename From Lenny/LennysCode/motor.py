#!/usr/bin/python

""" This just holds a few convenience functions for moving the motor using the RRB3 library """

from rrb3 import *
import time

rr = RRB3(11, 11)

def motor_forward(speed):
    if speed > 1:
        speed = 1
    if speed < 0:
        speed = 0
    rr.set_motors(speed, 0, speed, 0)

def motor_reverse(speed):
    if speed > 1:
        speed = 1
    if speed < 0:
        speed = 0
    rr.set_motors(speed, 1, speed, 1)
    
def motor_left(speed):
    pass

def motor_right(speed):
    pass

def motor_stop():
    rr.set_motors(0, 0, 0, 0)
