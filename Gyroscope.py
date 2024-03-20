import time
import serial
import board
import busio
import adafruit_bno055

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Initialize serial port
serial_port = '/dev/tty1'  # Update this to the correct serial port
baud_rate = 9600  # Serial communication baud rate
ser = serial.Serial(serial_port, baudrate=baud_rate, timeout=1)

print("Publishing BNO055 heading to serial port")

try:
    while True:
        # Get Euler angles (heading, roll, pitch)
        euler = sensor.euler
        if euler is not None:
            heading = euler[0]  # Extract the heading value
            print(f"Heading: {heading:.2f}")
            
            # Publish heading to serial port
            ser.write(f"{heading:.2f}\n".encode())
        
        # Delay to avoid spamming
        time.sleep(1)
except KeyboardInterrupt:
    print("Program terminated")
finally:
    # Close serial connection
    ser.close()
