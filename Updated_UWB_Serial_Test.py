import serial
from scipy.optimize import minimize
import numpy as np

class KalmanFilter:
    def __init__(self, A, B, H, Q, R, x0, P0):
        self.A = A  # State transition matrix
        self.B = B  # Control input matrix
        self.H = H  # Measurement matrix
        self.Q = Q  # Process noise covariance matrix
        self.R = R  # Measurement noise covariance matrix
        self.x = x0  # Initial state estimate
        self.P = P0  # Initial covariance estimate

    def predict(self, u=None):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.x.shape[0])
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

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
            A = np.eye(2)  # State transition matrix
            B = None  # Control input matrix
            H = np.eye(2)  # Measurement matrix
            Q = 0.01 * np.eye(2)  # Process noise covariance matrix
            R = 0.1 * np.eye(2)  # Measurement noise covariance matrix
            P0 = 0.1 * np.eye(2)  # Initial covariance estimate
            kf = KalmanFilter(A, B, H, Q, R, x0, P0)

            while True:
                uwb_distances_dict = {}
                for _ in range(4):
                    data = ser.readline().decode().strip()
                    if data:
                        anchor_id, distance = map(float, data.split(","))
                        uwb_distances_dict[anchor_id] = distance

                if len(uwb_distances_dict) == 4:
                    # Filter UWB readings using Kalman filter
                    z = np.array(list(uwb_distances_dict.values())).reshape(-1, 1)
                    kf.predict()
                    kf.update(z)

                    filtered_distances = kf.x.flatten()
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
