import board
import adafruit_bno055

def main():
    i2c = board.I2C()  
    sensor = adafruit_bno055.BNO055_I2C(i2c)  
      
    try:
        while True:
           yaw = sensor.euler[0] 
           print(yaw)
    except:
        exit()
    
   

if __name__ == '__main__':
    main()