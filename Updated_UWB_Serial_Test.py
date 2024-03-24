import serial
from scipy.optimize import minimize
import numpy as np

def location_solver(points, distances, x0):
    def objective_func(X):
        x, y = X
        # Calculate the error as the sum of squared differences between measured distances
        # and distances from (x, y) to each point
        error = sum([(distance - np.sqrt((x - point[0])**2 + (y - point[1])**2))**2 for point, distance in zip(points, distances)])
        return error
    
    # Perform the minimization to find the location (x, y) that minimizes the error
    result = minimize(objective_func, x0, method='L-BFGS-B')
    
    # Check the optimization result and return the found location or the initial guess
    if result.success and result.x[0] >= 0 and result.x[1] >= 0:
        return result.x
    else:
        return x0

if __name__ == "__main__":
    x0 = np.array([0, 0])  # Initial guess for the location
    points = {1: (0, 0), 2: (10, 0)}  # Points corresponding to UWB sensors indexed 1 and 2

    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Attempt to open the serial port
    except:
        print("Failed to open serial port. Please check the port and try again.")
    else:
        try:
            while True:
                uwb_distances_dict = {}
                # Read four lines from the serial port for the four UWB sensors
                for _ in range(4):
                    data = ser.readline().decode().strip()
                    if data:
                        anchor_id, distance = map(float, data.split(","))
                        # Store the distance if it corresponds to one of the two sensors of interest
                        if anchor_id in points:
                            uwb_distances_dict[anchor_id] = distance

                # Check if distances for both sensors of interest have been received
                if len(uwb_distances_dict) == 2:
                    distances = [uwb_distances_dict[anchor_id] for anchor_id in points]
                    target_location = location_solver(list(points.values()), distances, x0)
                    print("Target location:", target_location)
                    if isinstance(target_location, np.ndarray):
                        x0 = target_location  # Update the initial guess for the next iteration
                else:
                    print("Waiting for distances from UWB sensors 1 and 2.")

        except KeyboardInterrupt:
            print("\nExiting due to keyboard interrupt.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            ser.close()
            print("Serial port closed.")
