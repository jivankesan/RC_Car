import RPi.GPIO as GPIO
import controls
import time
import Gyroscope
import pigpio
import MotorEncoder
import serial
import math
import board
import adafruit_bno055

from motors import Car


if __name__ == "__main__":
    
    def read_yaw_angle(sensor):
        euler = sensor.euler[0]
        if euler is not None:
            return euler 
        return None

    def normalize_angle(angle):
        return angle % 360

    i2c = board.I2C()  
    sensor = adafruit_bno055.BNO055_I2C(i2c) 
    
    Pin1 = 8
    Pin2 = 25
    SAMPLE_TIME = 0.01
    
    pi = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    car = Car()
    
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    
    dist = 20

    pi = pigpio.pi()
    pi2 = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    
    points = [(0.1,0.1), (0.0,0.0), (0.2,0.2)]
    
    car = Car()
    curr_point = (0,0)
    curr_angle = read_yaw_angle(sensor) 
    
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def calculate_target_yaw(current_yaw, target_point, current_point):
        angle_to_target = math.atan2(target_point[1] - current_point[1], target_point[0] - current_point[0]) * 180 / math.pi
        return normalize_angle(angle_to_target - current_yaw)
    
    try:
        for point in points:
            dist = distance(curr_point, point)
            target_yaw = calculate_target_yaw(curr_angle, point, curr_point)
            
            if target_yaw < 0:
                car.drive(3)  
            else:
                car.drive(2) 
            
            while True:
                current_yaw = read_yaw_angle(sensor)
                if current_yaw is None:
                    continue  # Skip iteration if sensor read failed
                    
                if abs(normalize_angle(current_yaw - target_yaw)) < 3:  # 5 degrees tolerance
                    break  # Exit loop once close to the target yaw
                    
            car.stop()
                   
            p.pulse_count=0 
                
            car.drive(0)
            while (p.pulse_count < 4685*(dist/0.471234)):
                curr_distance = (p.pulse_count/4685)*0.471234
                print(curr_distance)

            print(point)
                
            curr_angle = read_yaw_angle(sensor)
            curr_point = point
            
        car.stop()
        print(points[-1])

    except KeyboardInterrupt:
        car.stop()
    # Cleanup GPIO when program is interrupted
        GPIO.cleanup() 
    
      