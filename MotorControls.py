import RPi.GPIO as GPIO
import controls
import time
import Gyroscope

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
    car = Car()
    gyro = Gyroscope.GYRO()
    try:
        while True:
            car.drive(2)
            time.sleep(1.932)
            car.drive(0)
            time.sleep(40)
            
            car.drive(0)
            time.sleep(10)
            roll, pitch, yaw = gyro.get_orientation(time.time())
            print(f"Roll={roll:.2f}° Pitch={pitch:.2f}° Yaw={yaw:.2f}°")
            
            
    except KeyboardInterrupt:
        # Cleanup GPIO when program is interrupted
        GPIO.cleanup()