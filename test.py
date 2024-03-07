#!/usr/bin/env python

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import smbus
import time
import math

# MPU6050 registers and their addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6b
SMPLRT_DIV = 0x19
CONFIG = 0x1a
GYRO_CONFIG = 0x1b
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3b
ACCEL_YOUT_H = 0x3d
ACCEL_ZOUT_H = 0x3f
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize the I2C bus
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# MPU6050 initialization
def mpu6050_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up the MPU-6050
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)  # Set sample rate to 1 kHz / (1 + 7) = 125 Hz
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)  # Disable FSYNC, set 260 Hz Acc filtering, 1 kHz Gyro sampling
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 24)  # Set gyro range to ±2000 deg/s
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)  # Set interrupt enable

# Read raw values from the sensor
def read_raw_data(addr):
    # Accel and gyro values are 16-bit
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    # Concatenate higher and lower value
    value = ((high << 8) | low)
    # To get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

# Convert the raw data to roll, pitch, and yaw
def get_motion_data():
    # Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    # Full scale range ±250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    
    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0
    
    roll = math.atan2(Ay, Az) * 57.3
    pitch = math.atan2(-Ax, math.sqrt(Ay * Ay + Az * Az)) * 57.3
    
    # Integration or complementary filter can be used here to compute yaw.
    yaw = Gz  # This is a simplification and may not give accurate yaw values.
    
    return roll, pitch, yaw

# Existing OpenGL setup and draw functions go here
# Modify the draw() function to use roll, pitch, and yaw from get_motion_data()

def main():
    pygame.init()
    display = (640, 480)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    mpu6050_init()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        roll, pitch, yaw = get_motion_data()
        # Modify the OpenGL drawing code to visualize roll, pitch, yaw
        # For simplicity, this code segment is omitted

        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == "__main__":
    main()
