import serial
from scipy.optimize import minimize
import numpy as np  # Import NumPy here

class KalmanFilter:
    def __init__(self, A, H, Q, R, x0, P0):
        self.A = A  # State transition matrix
        self.H = H  # Measurement matrix
        self.Q = Q  # Process noise covariance matrix
        self.R = R  # Measurement noise covariance matrix
        self.x = x0.reshape(-1, 1)  # Initial state estimate as column vector
        self.P = P0  # Initial covariance estimate

    def predict(self):
        # Predict state
        self.x = np.dot(self.A, self.x)
        # Predict covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x, self.P  # Return the predicted state and covariance

    def update(self, z):
        # Calculate predicted measurement
        z_pred = np.dot(self.H, self.x)
        # Calculate innovation
        y = z - z_pred
        # Calculate innovation covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        # Calculate Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Update state estimate
        self.x = self.x + np.dot(K, y)
        # Update covariance estimate
        self.P = np.dot(np.eye(self.P.shape[0]) - np.dot(K, self.H), self.P)

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
            # Kalman filter parameters
            dt = 1  # Time step
            A = np.eye(2)  # State transition matrix (identity matrix since there's no control input)
            H = np.eye(2)  # Measurement matrix
            Q = 0.001 * np.eye(2)  # Adjust the values as needed
            R = 0.5 * np.eye(2)  # Measurement noise covariance matrix
            P0 = 0.1 * np.eye(2)  # Initial covariance estimate
            kf = KalmanFilter(A, H, Q, R, x0, P0)

            while True:
                uwb_distances_dict = {}
                for _ in range(4):
                    data = ser.readline().decode().strip()
                    if data:
                        anchor_id, distance = map(float, data.split(","))
                        uwb_distances_dict[anchor_id] = distance

                if len(uwb_distances_dict) == 4:
                    # Filter UWB readings using Kalman filter
                    predicted_state, predicted_covariance = kf.predict()
                    kf.update(np.array(list(uwb_distances_dict.values())).reshape(-1, 1))

                    filtered_distances = predicted_state.flatten()
                    print("Filtered distances:", filtered_distances)

                    # Solve using first two points and distances
                    distances1 = filtered_distances[:2]
                    solution1 = location_solver(list(points_group_1.values()), distances1, x0)
                    
                    print(distances1)
                    print(solution1)
                    
                    # Solve using next two points and distances
                    distances2 = filtered_distances[2:]
                    solution2 = location_solver(list(points_group_2.values()), distances2, x0)

                    print(distances2)
                    print(solution2)
                    
                    # Calculate the average of the solutions
                    if isinstance(solution1, np.ndarray) and isinstance(solution2, np.ndarray):
                        final_solution = (solution1 + solution2) / 2
                        print("Final target location:", final_solution)
                        x0 = final_solution.flatten()  # Update the initial guess
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