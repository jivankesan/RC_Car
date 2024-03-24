import serial
from scipy.optimize import minimize
import numpy as np

def location_solver(points, distances, x0):
    def objective_func(X):
        x, y = X
        # Calculate the error as the sum of squared differences between measured distances and distances from (x, y) to each point
        error = sum([(distance - np.sqrt((x - point[0])**2 + (y - point[1])**2))**2 for point, distance in zip(points, distances)])
        return error
    
    # Perform the minimization to find the location (x, y) that minimizes the error
    result = minimize(objective_func, x0, method='L-BFGS-B')
    
    if result.success and result.x[0] >= 0 and result.x[1] >= 0:
        return result.x
    else:
        return x0

if __name__ == "__main__":
    x0 = np.array([0, 0])  # Initial guess for the location
    points_group_1 = {1: (0, 0), 2: (7, 0)}
    points_group_2 = {3: (0, 7), 4: (7, 7)}

    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    except:
        print("Failed to open serial port. Please check the port and try again.")
    else:
        try:
            while True:
                uwb_distances_dict = {}
                for _ in range(4):
                    data = ser.readline().decode().strip()
                    if data:
                        anchor_id, distance = map(float, data.split(","))
                        uwb_distances_dict[anchor_id] = distance

                if len(uwb_distances_dict) == 4:
                    # Solve using first two points and distances
                    distances1 = [uwb_distances_dict[id] for id in points_group_1]
                    solution1 = location_solver(list(points_group_1.values()), distances1, x0)
                    
                    print(distances1)
                    print(solution1)
                    
                    # Solve using next two points and distances
                    distances2 = [uwb_distances_dict[id] for id in points_group_2]
                    solution2 = location_solver(list(points_group_2.values()), distances2, x0)

                    print(distances2)
                    print(solution2)
                    
                    # Calculate the average of the solutions
                    if isinstance(solution1, np.ndarray) and isinstance(solution2, np.ndarray):
                        final_solution = (solution1 + solution2) / 2
                        print("Final target location:", final_solution)
                        x0 = final_solution  # Update the initial guess
                    else:
                        print("Could not compute a valid location for one of the groups.")
                else:
                    print("Waiting for distances from all UWB sensors.")

        except KeyboardInterrupt:
            print("\nExiting due to keyboard interrupt.")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            ser.close()
            print("Serial port closed.")
