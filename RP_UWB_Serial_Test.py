import serial
from scipy.optimize import minimize
import math
import numpy as np

def compute_and_publish_location(point1, point2, distance12, point3, point4, distance34):
        uwb1_position = location_solver(point1, point2, distance12)
        uwb2_position = location_solver(point3, point4, distance34)
        if isinstance(uwb1_position, str) or isinstance(uwb2_position, str):  # Check if the return value indicates an error
            print("ERROR")
            return 

        x, y = (uwb1_position[0] + uwb2_position[0])/2, (uwb1_position[1] + uwb2_position[1])/2 

        # Once computed, publish the current location
        print(x)
        print(y)
    
def location_solver(points, distances, x0):
    # Adjusted objective function to minimize
    def objective_func(X):
        x, y = X
        return sum([((x - point[0])**2 + (y - point[1])**2 - d**2)**2 for point, d in zip(points, distances)])
    
    # Perform the minimization with adjusted objective function
    result = minimize(objective_func, x0, method='L-BFGS-B')
        
    if result.success:
        # Check if the solution coordinates are reasonable, adjust as necessary
        if result.x[0] >= 0 and result.x[1] >= 0:
            return result.x
        else:
            return "Solution has non-positive coordinates."
    else:
        return "Optimization failed."


if __name__ == "__main__":

    x0 = np.array([0,0])
    points = [(0,0), (10,0), (0,10), (10,10)]
    
    
    # Adjust port, baud rate, and timeout as needed
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    except:
        print("Failed to open serial port. Please check the port and try again.")
    else:
        try:
            while True:
                distances = []
                for i in range(0,4):
                    # Read data from the serial port
                    data = ser.readline().decode().strip()
                    if data:  # Only print if data is not empty
                        print("Received:", data)
                    distances.append(data)
                    # adjust order of points based on the uwb location accordingly
                print(len(distances))
                target_location = location_solver(points, distances, x0)
                print("Target location:", target_location)
                    
                   
        except KeyboardInterrupt:
            print("\nExiting due to keyboard interrupt.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            ser.close()  # Ensure the serial port is closed
            print("Serial port closed.")
