import serial

# Adjust port, baud rate, and timeout as needed
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except:
    print("Failed to open serial port. Please check the port and try again.")
else:
    try:
        while True:
            # Read data from the serial port
            data = ser.readline().decode().strip()
            if data:  # Only print if data is not empty
                print("Received:", data)
    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()  # Ensure the serial port is closed
        print("Serial port closed.")
