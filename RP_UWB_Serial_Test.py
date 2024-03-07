import serial

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port and baud rate as needed

try:
    while True:
        # Read data from the serial port
        data = ser.readline().decode().strip()
        print("Received:", data)  # Print received data
except KeyboardInterrupt:
    ser.close()  # Close the serial port on Ctrl+C
