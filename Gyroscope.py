import time
import board
import busio
from adafruit_bno055 import BNO055
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)

sensor = BNO055.BNO055()

while True:
    euler = sensor.euler
    print('X: {:.2f}, Y: {:.2f}, Z: {:.2f}'.format(euler[0], euler[1], euler[2]))
    time.sleep(0.1)

