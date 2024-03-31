import RPi.GPIO as GPIO
import time
import math
import board
import adafruit_bno055
from motors import Car
import pigpio
import MotorEncoder

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
    SAMPLE_TIME = 0.01
    
    car = Car()
    curr_point = (0, 0)
    curr_angle = read_yaw_angle(sensor) 
    
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def calculate_target_yaw(current_yaw, target_point, current_point):
        angle_to_target = math.atan2(target_point[1] - current_point[1], target_point[0] - current_point[0]) * 180 / math.pi
        return normalize_angle(angle_to_target - current_yaw)
    
    try:
        points = [(0.1, 0.1), (0.2, 0.2), (0.2, 0.5)]
        
        for point in points:
            target_yaw = calculate_target_yaw(curr_angle, point, curr_point)
            
            if target_yaw < -90:  # Turn left for 180 degrees left
                car.drive(2)  # Turn left
            elif target_yaw > 90:  # Turn right for 180 degrees right
                car.drive(3)  # Turn right
            else:
                car.stop()  # No turning needed
            
            while True:
                current_yaw = read_yaw_angle(sensor)
                if current_yaw is None:
                    continue  # Skip iteration if sensor read failed
                    
                if abs(normalize_angle(current_yaw - target_yaw)) < 3:  # 5 degrees tolerance
                    break  # Exit loop once close to the target yaw
                    
            car.stop()
            
            dist = distance(curr_point, point)
            pi = pigpio.pi()
            p = MotorEncoder.reader(pi, Pin1)
            p.pulse_count = 0
            car.drive(1)  # Drive forward
            while p.pulse_count < 4685 * (dist / 0.471234):
                curr_distance = (p.pulse_count / 4685) * 0.471234
                print(curr_distance)
            
            curr_angle = read_yaw_angle(sensor)
            curr_point = point
            
        car.stop()
        print("Reached the final destination")

    except KeyboardInterrupt:
        car.stop()
        GPIO.cleanup()  # Cleanup GPIO when program is interrupted