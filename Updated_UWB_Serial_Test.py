import serial
import numpy as np
from scipy.optimize import minimize

def location_solver(points, distances, x0):
    # Adjusted objective function to minimize
    def objective_func(X):
        x, y = X
        error = sum([(distance - ((x - point[0])**2 + (y - point[1])**2)**0.5)**2 for point, distance in zip(points, distances)])
        return error
    
    # Perform the minimization with adjusted objective function
    result = minimize(objective_func, x0, method='L-BFGS-B')
        
    if result.success:
        # Check if the solution coordinates are reasonable, adjust as necessary
        if result.x[0] >= 0 and result.x[1] >= 0:
            return result.x
        else:
            return "Solution has non-positive coordinates."
    else:
        return x0

if __name__ == "__main__":
    x0 = np.array([0, 0])
    points = [(0, 0), (4, 0), (0, 4), (4, 4)]
    
    # Open serial port
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    except Exception as e:
        print("Failed to open serial port. Please check the port and try again.")
        exit()

    try:
        while True:
            distances = [0.0] * 4  # Initialize distances list
            uwb_distances_dict = {}

            for i in range(0, 4):
                # Read data from the serial port
                data = ser.readline().decode().strip()
                if data:  # Only process if data is not empty
                    data = data.split(",")
                    anchor_id = int(data[0])
                    try:
                        distance = float(data[1].split('\r')[0])  # Remove any trailing characters
                        uwb_distances_dict[anchor_id] = distance
                        distances[anchor_id - 1] = distance  # Reorder distances based on anchor ID
                    except ValueError:
                        print("Invalid data format:", data)

            # Prepare distances in desired order (1, 2, 3, 4)
            distances = [distances[i] for i in range(4)]

            print("Distances dictionary:", uwb_distances_dict)
            print("Distances count:", len(distances))
            
            # Localize the target location based on distances
            target_location = location_solver(points, distances, x0)
            print("Target location:", target_location)
            x0 = target_location

    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()  # Ensure the serial port is closed
        print("Serial port closed.")
