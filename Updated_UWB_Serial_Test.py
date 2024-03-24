import serial
import numpy as np
from scipy.optimize import minimize

# Define the positions of the UWB anchors
anchor_positions = [(0, 0), (10, 0), (0, 10), (10, 10)]

def calculate_distances(robot_position):
    # Calculate distances from the robot to each anchor
    distances = [np.sqrt((robot_position[0] - anchor[0])**2 + (robot_position[1] - anchor[1])**2) for anchor in anchor_positions]
    return distances

def location_solver(distances, x0):
    # Objective function to minimize
    def objective_func(X):
        x, y = X
        error = sum([(np.sqrt((x - anchor[0])**2 + (y - anchor[1])**2) - distance)**2 for anchor, distance in zip(anchor_positions, distances)])
        return error
    
    # Perform the minimization with the objective function
    result = minimize(objective_func, x0, method='Nelder-Mead')
        
    if result.success:
        # Check if the solution coordinates are reasonable, adjust as necessary
        if result.x[0] >= 0 and result.x[1] >= 0:
            return result.x
        else:
            return "Solution has non-positive coordinates."
    else:
        return x0

if __name__ == "__main__":
    x0 = np.array([0, 0])  # Initial guess for the robot's position
    
    # Adjust port, baud rate,
