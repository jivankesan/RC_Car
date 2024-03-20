import RPi.GPIO as GPIO
from motors import Car
import pigpio
import MotorEncoder
import serial



if __name__ == "__main__":
    GPIO.setmode(GPIO.BOARD)
     
    sensor = adafruit_bno055.BNO055_I2C(i2c) 
    
    Pin1 = 8
    Pin2 = 25
    RUN_TIME = 60.0
    SAMPLE_TIME = 0.01
    
    pi = pigpio.pi()
    p = MotorEncoder.reader(pi, Pin1)
    car = Car()
    
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    
    dist = 20
        
    try:
        while True:
            car.drive(0)
            angle = sensor.euler[0]
            curr_distance = (p.pulse_count/4685)*0.471234
            data = ser.readline().decode().strip()
            print(f"Angle: {angle}, Current Distance: {curr_distance}, Data: {data}")
            
    except KeyboardInterrupt:
        print("stopped")
    finally:
        car.stop()
        GPIO.cleanup()
        