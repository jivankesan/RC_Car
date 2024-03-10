import RPi.GPIO as GPIO
import controls
import time
import Gyroscope
import pigpio
import MotorEncoder
import serial
import math

class Car():
    def __init__(self):
        # Define GPIO pins for motors
        self.PWM_RB = 36  
        self.DIR_RB = 32
        
        self.PWM_LB = 31
        self.DIR_LB = 29
        
        self.PWM_RF = 13
        self.DIR_RF = 11
        
        self.PWM_LF = 16
        self.DIR_LF = 18

        self.pins = [self.PWM_RF, self.DIR_RF, self.PWM_LB, self.DIR_LB, self.PWM_RB, self.DIR_RB, self.PWM_LF, self.DIR_LF]
        self.pwm_pins = [self.PWM_LB, self.PWM_LF, self.PWM_RB, self.PWM_RF]
        self.dir_L = [self.DIR_LB, self.DIR_LF]
        self.dir_R = [self.DIR_RB, self.DIR_RF]
 

        # Setup GPIO pins
        GPIO.setmode(GPIO.BOARD)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)    


    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)
    
    def turn_right(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_L:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
    
    def turn_left(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_R:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)

    def forward(self):
        self.stop()
        time.sleep(.1)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
   
    def reverse(self):
        self.stop()
        time.sleep(.1)
        for pin in self.dir_R:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.dir_L:
            GPIO.output(pin, GPIO.HIGH)
        for pin in self.pwm_pins:
            GPIO.output(pin, GPIO.HIGH)
        
        
    def drive(self, axis_data):
    
        # mapping forwards backwards movement
        if axis_data == 2:  
            self.turn_left()
        
        elif axis_data == 3:  
            self.turn_right()
            
        elif axis_data == 0: 
            self.forward()
        
        elif axis_data == 1: 
            self.reverse()
        
        else: self.stop() 


if __name__ == "__main__":
    
    Pin1 = 8
    Pin2 = 25
    RUN_TIME = 60.0
    SAMPLE_TIME = 0.01
    
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port and baud rate as needed

    pi = pigpio.pi()
    pi2 = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    p2 = MotorEncoder.reader(pi2, Pin2)
    
    points = [(0.3,0.3),(0.6,1.2),(0.6,1.5),(1.8,2.1),(2.1,2.1),(2.4,2.4)]
    
    car = Car()
    curr_angle = 0
    curr_point = (0,0)
    
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    def calculate_travel_angle(point1, point2, current_angle):

        # Calculate the angle to destination
        angle_to_destination = math.atan2(point2[1] - point1[1], point2[0] - point1[0]) * (180 / math.pi)
        
        # Calculate the required angle adjustment from the current orientation
        angle_adjustment = angle_to_destination - current_angle
        
        # Normalize the angle to the range [-180, 180]
        angle_adjustment = (angle_adjustment + 180) % 360 - 180
        
        return angle_adjustment
    
    try:
        for point in points:
            dist = distance(curr_point, point)
            angle_to_turn = calculate_travel_angle(curr_point, point, curr_angle)
                
            seconds_per_degree = 1.947 / 90  # Time it takes to turn one degree
            turn_duration = abs(angle_to_turn) * seconds_per_degree                
            if angle_to_turn < 0:
                car.drive(2)     
            else:
                car.drive(3)
            time.sleep(turn_duration)      
            car.stop()
                   
            p.pulse_count=0 
                
            car.drive(0)
            while (p.pulse_count < 4685*(dist/0.471234)):
                curr_distance = (p.pulse_count/4685)*0.471234
                print(curr_distance)

            print('reached: {point}')
                
            curr_angle = (curr_angle + angle_to_turn) % 360
            curr_point = point
        car.stop()
        print(points[-1])
           
    except KeyboardInterrupt:
        # Cleanup GPIO when program is interrupted
        GPIO.cleanup()