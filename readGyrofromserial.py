import serial
import time

# Initialize serial port - update to the correct serial port for your receiver device
serial_port = '/dev/serial0'  # This needs to be the serial port on your receiver device
baud_rate = 9600  # Make sure this matches the baud rate used for sending the data

# Setup serial connection
ser = serial.Serial(serial_port, baudrate=baud_rate, timeout=1)

print("Listening for gyro heading values on serial port...")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()  # Read a line and decode it from bytes to string
            try:
                heading = float(line)  # Try to convert the line to a float
                print(f"Received heading: {heading}")
            except ValueError:
                # If conversion fails, print the error and ignore the line
                print(f"Could not convert received line to float: '{line}'")
        else:
            # No data waiting, let's not spam the CPU
            time.sleep(0.1)
except KeyboardInterrupt:
    print("Program terminated")
finally:
    # Close serial connection
    ser.close()
