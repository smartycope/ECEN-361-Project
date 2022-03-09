import smbus
from math import degrees, atan2
import time
from MPU6050 import MPU6050
from PID import PID
import motor
# import standup
from Util import dist
import os
from datetime import datetime
# from threading import Timer
from time import time as now
from math import pi


# Don't touch these
address = 0x68  # This is the address value read via the i2cdetect command
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards
sensor = MPU6050(bus, address, "MPU6050")


# This is important
# Use this link to tune the parameters:
# http://www.grauonline.de/alexwww/ardumower/pid/pid.html

# pid=PID(2.14, .74, 1.08) # looks decent, with a just little bit of overcorrection
pid=PID(1.88, .28, 1.19) # looks pretty good, more stochastic, but smooth output with no overcorrection
# pid=PID(1.0, -0.04, 0.0) # What we came with
# pid=PID(2, 0, 1) # The default we found online
pid.setPoint(0.0)

# The comp filter tracks its own time
dtTimer = now()

# For the comp filter
yaw = 0
pitch = 0
roll = 0
gyroRoll = 0
gyroPitch = 0
gyroYaw = 0

# For emergencies
s=motor.stop

# Because too much data is annoying
printdelay = 0
DELAY = 20

# Specific pauses for the standup function
pause1=p1=.6
pause2=p2=.5 # was .294
pause25=p25=.1
pause3=p3=.1
joltAngle=-.2
checkPause=.05

# This is how close we care to be to the target angle (we think) -- actually probably not that?
complimenteryFilterConst = .98 # was .98

# This is the speed multiplier
pidMultiplier = .055
# How close to the top we need to be before we stop caring
balancedTolerance = .005

# How close to falling over we need to be before we just give up
motorKillTolerance = 0.1
# The face down and face up roll angles
faceUpRollVal = 1.52
faceDownRollVal = -1.45

# How many consecutive readings we need to get that we've fallen over before we believe them
killReadingsReq = 5
killReadingCount = 0

# Tread carefully
manualRollOffset = .0

# How often we're running the main loop (in milliseconds)
loopDelay = 5

# Runs a function on a loop for a specific amount of time
def psleep(seconds, func):
    end = now()
    while now() < end:
        func()


def closeEnough(a, b, tolerance):
    """ Returns True if a is within tolerance range of b """
    return a <= b + tolerance and a >= b - tolerance


def compFilter():
    global yaw, pitch, roll, dtTimer, gyroRoll, gyroPitch, gyroYaw, printdelay, DELAY

    # Get delta time and record time for next call
    dt = now() - dtTimer
    dtTimer = now()

    sensor.read_accel_data()
    sensor.read_gyro_data()

    # Acceleration vector angle
    # accPitch = degrees(atan2(sensor.accel_scaled_y, sensor.accel_scaled_z))
    accRoll  = degrees(atan2(sensor.accel_scaled_x, sensor.accel_scaled_z)) # add PI?

    # Gyro integration angle
    # gyroRoll -= sensor.gyro_scaled_y * dt
    # gyroRoll += sensor.gyro_scaled_y * dt
    # gyroPitch += sensor.gyro_scaled_x * dt
    # gyroYaw += sensor.gyro_scaled_z * dt
    # yaw = gyroYaw

    # yaw = sensor.gyro_scaled_z * dt

    # Comp filter
    roll  = (complimenteryFilterConst)*(roll  - sensor.gyro_scaled_y * dt) + (1-complimenteryFilterConst)*(accRoll)
    # pitch = (complimenteryFilterConst)*(pitch + sensor.gyro_scaled_x * dt) + (1-complimenteryFilterConst)*(accPitch)

    # Print data
    # if not printdelay % DELAY:

        # print(" R: " + str(round(roll,1)))
        # print()
            # + " P: " + str(round(pitch,1)) \
            # + " Y: " + str(round(yaw,1)))

    # return (yaw, pitch, roll)
    return roll


def main():
    global printdelay

    try:
        while True:
            time.sleep(loopDelay / 1000)

            printdelay += 1

            # Get the adjusted values from the complimentary filter
            # adjYaw, adjPitch, adjRoll = compFilter(time_diff)
            adjRoll = compFilter() + manualRollOffset
            # Put the adjusted roll value into the PID
            speed = pid.update(adjRoll) * pidMultiplier

            # We're idiots
            if not printdelay % DELAY:
                print('adjRoll:', round(adjRoll, 2))
                print('speed:', round(speed, 2))
                print('pid raw:', round(speed / pidMultiplier, 2))
                print()

            # First test if we're laying down -- if so, just give up
            if closeEnough(adjRoll, faceUpRollVal, motorKillTolerance):
                killReadingCount += 1
                if killReadingCount > killReadingsReq:
                    break
            elif closeEnough(adjRoll, faceDownRollVal, motorKillTolerance):
                killReadingCount += 1
                if killReadingCount > killReadingsReq:
                    break

            # Otherwise, move accordingly
            elif speed < 0 and not closeEnough(speed, 0, balancedTolerance):
                motor.forward(abs(speed))
                killReadingCount = 0
            elif speed > 0 and not closeEnough(speed, 0, balancedTolerance):
                motor.reverse(abs(speed))
                killReadingCount = 0

            # We need no adjustment
            else:
                killReadingCount = 0
                # motor.stop()

    # Make sure the motors stop when we kill it
    finally:
        motor.stop()


def standup():
    # First get if we're face down or face up
    sensor.update()
    faceUp = sensor.roll > 0

    forward = int(not faceUp)
    backward = int(faceUp)
    rollSide = 1 if faceUp else -1

    # Using this instead of motor.raw.forward cause I think it's slightly faster for some reason
    # Just kidding, it's not, but I don't want to rewrite it
    print('Going forward')
    motor.raw.set_motors(1, forward, 1, forward)
    time.sleep(pause1)

    print('Going backward')
    motor.raw.set_motors(1, backward, 1, backward)
    time.sleep(pause2)

    main()
