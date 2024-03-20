import time
# import board
# import busio
# import adafruit_bno055
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)

res = GPIO.getmode()

print(res)



"""
    
    
 
res = GPIO.getmode()
print(res)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

while True:
    euler = sensor.euler
    print('X: {:.2f}, Y: {:.2f}, Z: {:.2f}'.format(euler[0], euler[1], euler[2]))
    time.sleep(0.1)

   """