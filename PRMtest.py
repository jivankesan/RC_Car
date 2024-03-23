from goal_manager import GoalManager
from pathplanner import PathPlanner
from MotorControls import Car
from MotorEncoder import reader

import serial
import json
import pandas as pd
import csv
from shapely.geometry import Point, LineString, Polygon


class parsecsv:
    def parse(self, csv_name):
        """
        Parse function that will execute parse a desired CSV file.
        """
        base_file_location = './csvfiles'
        csv_file_path = base_file_location + csv_name + '.csv'
        
        array = []
        return self.parse_csv(csv_file_path, csv_name)


    def parse_csv(self, file_path, csv_file_name):
        """
        CSV Parse function that attempts to parse through the CSV files.
        """
        try:
            # Use pandas to read the CSV file into a DataFrame
            df = pd.read_csv(file_path)
            
            # Converting Pandas Datarame to JSON and sending as HTTP Header
            records_list = df.to_dict(orient='records')
            
            # Arrayifying records
            return self.arrayify_records(records_list, csv_file_name)

        except FileNotFoundError:
            print(f"CSV file not found: {file_path}")



if __name__ == "__main__":
    
    checkpoints = ''
    obstacles = ''
    environment = ''
    
    points = []
    
    Pin1 = 8
    Pin2 = 25
    RUN_TIME = 60.0
    SAMPLE_TIME = 0.01
    
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port and baud rate as needed

    pi = pigpio.pi()
    pi2 = pigpio.pi()
    p = reader(pi, Pin1)
    p2 = reader(pi2, Pin2)
    
    car = Car()
    path_planner = PathPlanner()
    CurrGoals = GoalManager()
    
    path_planner.checkpoints = compute_gate_midpoints(checkpoints)
    path_planner.environment = parse_csv_for_points_and_boundary(environment)
    path_planner.obstacles = read_obstacles(obstacles)
    
    print("running path planner")
    path_planner.global_prm()
    print("finished path planner")
    
    while path_planner.index < path_planner.maxlen:
        
    
