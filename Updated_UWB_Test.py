import serial

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except:
    print("Failed to open serial port. Please check the port and try again.")
else:
    try:
        tag_data = {}  #Initialize empty dictionary to store UWB data
        while True:
            data = ser.readline().decode().strip() #Read data from the serial port
            if data:  #Only process if data is not empty
                tag, distance = map(float, data.split(','))
                tag_data[tag] = distance  #Store distance data in the dictionary
                print("Received:", tag_data) 
    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()  #Ensure the serial port is closed
        print("Serial port closed.")