import smbus
from math import degrees, atan2
import time
from MPU6050 import MPU6050
from PID import PID
import motor
import standup
from Util import dist
import os
from datetime import datetime
from threading import Timer


# print(i2c_raspberry_pi_bus_number())
timerDurration = 8
pidMultiplier = 30

# These aren't used?
gyro_scale = 131.0
accel_scale = 16384.0

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846

address = 0x68  # This is the address value read via the i2cdetect command
bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards

now = time.time()

# Not sure what these are... when you figure out change the name to make more sense please
complimenteryFilterConst = 0.98

time_diff = 0.01

sensor = MPU6050(bus, address, "MPU6050")
sensor.read_raw_data() # Reads current data from the sensor

def get_y_rotation(x,y,z):
    return -atan2(x, dist(y,z))

def get_x_rotation(x,y,z):
    return atan2(y, dist(x,z))

# Created to end program after testing for n seconds
def exitfunc():
    print("Exit Time", datetime.now())
    os._exit(0)
    # Motors run forwards after ending call, just type python startup.py to reset them?

# This for loop doesn't seem like it would run the code indefinetly. Shouldn't there be a While True loop here? Give it a try
def balance():
    rate_gyroX = 0.0
    rate_gyroY = 0.0
    rate_gyroZ = 0.0
    
    gyroAngleX = 0.0
    gyroAngleY = 0.0
    gyroAngleZ = 0.0
    
    raw_accX = 0.0
    raw_accY = 0.0
    raw_accZ = 0.0
    
    rate_accX = 0.0
    rate_accY = 0.0
    rate_accZ = 0.0
    
    accAngX = 0.0
    
    # Final Angle(s)
    CFangleX = 0.0
    CFangleX1 = 0.0
    
    FIX = -12.89
    
    for i in range(0, int(300.0 / time_diff)):
        time.sleep(time_diff - 0.005)

        sensor.read_raw_data()
        # Gyroscope value Degree Per Second / Scalled Data
        rate_gyroX = sensor.read_gyro_scaled_x()
        rate_gyroY = sensor.read_gyro_scaled_y()
        rate_gyroZ = sensor.read_gyro_scaled_z()

        # The angle of the Gyroscope
        gyroAngleX += rate_gyroX * time_diff
        gyroAngleY += rate_gyroY * time_diff
        gyroAngleZ += rate_gyroZ * time_diff

        # Accelerometer Raw Value
        raw_accX = sensor.read_accel_raw_x()
        raw_accY = sensor.read_accel_raw_y()
        raw_accZ = sensor.read_accel_raw_z()

        # Accelerometer value Degree Per Second / Scalled Data
        rate_accX = sensor.read_accel_scaled_x()
        rate_accY = sensor.read_accel_scaled_y()
        rate_accZ = sensor.read_accel_scaled_z()

        # We're not using this later on... what does this do?
        # http://ozzmaker.com/2013/04/18/success-with-a-balancing-robot-using-a-raspberry-pi/
        # In the link, K == AA, CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
        accAngX = ( atan2(rate_accX, rate_accY) + M_PI ) * RAD_TO_DEG
        CFangleX = complimenteryFilterConst * ( CFangleX + rate_gyroX * time_diff) + (1 - complimenteryFilterConst) * accAngX


        # http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
        accAngX1 = degrees(get_x_rotation(rate_accX, rate_accY, rate_accX))
        CFangleX1 = ( complimenteryFilterConst * ( CFangleX1 + rate_gyroX * time_diff) + (1 - complimenteryFilterConst) * accAngX1 )

        # print(accAngX)
        print(accAngX1)

        # Followed the Second example because it gives resonable pid reading
        # Try using both CFangleX and CFangleX1?
        pid = int(p.update(CFangleX1))
        speed = pid * pidMultiplier

        if(pid > 0):
            motor.forward(speed)
        elif(pid < 0):
            motor.reverse(abs(speed))
        else:
            motor.stop()

##################################################################################          
####  MAIN CODE ###

Timer(timerDurration, exitfunc).start() # exit in timerDurration seconds
print("starting")

p=PID(1.0,-0.04,0.0)
p.setPoint(0.0)  

#standup.standUp()
balance()


##################################################################################
