import time
import math
import board
import adafruit_bno055
import pigpio
import MotorEncoder
from motors import Car

def read_yaw_angle(sensor):
    euler = sensor.euler[0]
    if euler is not None:
        return euler 
    return None

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

def calculate_target_yaw(current_yaw, target_point, current_point):
    angle_to_target = math.atan2(target_point[1] - current_point[1], target_point[0] - current_point[0]) * 180 / math.pi
    target_yaw = normalize_angle(angle_to_target - current_yaw)
    direction = 'left' if target_yaw < 0 else 'right'
    return abs(target_yaw), direction

if __name__ == "__main__":
    i2c = board.I2C()  
    sensor = adafruit_bno055.BNO055_I2C(i2c) 

    Pin1 = 8
    SAMPLE_TIME = 0.01
    
    pi = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    car = Car()
    
    points = [(0.3,0.3),(1.2,0.6)]
    
    curr_point = (0,0)
    curr_angle = read_yaw_angle(sensor) 
    
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    try:
        for point in points:
            dist = distance(curr_point, point)
            target_yaw, direction = calculate_target_yaw(curr_angle, point, curr_point)
            
            if direction == 'left':
                car.turn_left()
            else:
                car.turn_right() 
            
            while True:
                current_yaw = read_yaw_angle(sensor)
                if current_yaw is None:
                    continue  # Skip iteration if sensor read failed
                    
                if abs(normalize_angle(current_yaw - target_yaw)) < 3:  # 5 degrees tolerance
                    break  # Exit loop once close to the target yaw
                    
            car.stop()
                   
            p.pulse_count = 0 
                
            car.forward()
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
