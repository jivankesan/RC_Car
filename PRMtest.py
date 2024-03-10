from goal_manager import GoalManager
from pathplanner import PathPlanner
from MotorControls import Car
from MotorEncoder import reader

import pigpio
import RPi.GPIO
import serial
import json
import pandas as pd
import csv
from shapely.geometry import Point, LineString, Polygon


def compute_gate_midpoints(csv_file_path):
    checkpoints = []

    with open(csv_file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        rows = list(reader)

        for i in range(0, len(rows), 2):
            gate_side_1 = rows[i]
            gate_side_2 = rows[i+1]
            midpoint_easting = (float(gate_side_1[1]) + float(gate_side_2[1])) / 2
            midpoint_northing = (float(gate_side_1[2]) + float(gate_side_2[2])) / 2
            checkpoint = Point(midpoint_easting, midpoint_northing)
            checkpoints.append(checkpoint)

    return checkpoints

def read_obstacles(csv_file_path):
    obstacles = []
    
    with open(csv_file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Create a Point instance using easting and northing, then buffer it
            point = Point(float(row['easting']), float(row['northing']))
            buffered_point = point.buffer(float(row['boundingRadius']))
            obstacles.append(buffered_point)
            
    return obstacles

def parse_csv_for_points_and_boundary(csv_file_path):
    points = {}
    with open(csv_file_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            point = Point(float(row['easting']), float(row['northing']))
            points[row['name']] = point

    # Assuming "origin" is the start point and "finish" is explicitly defined
    start = points.get("origin", Point(0, 0))
    finish = points.get("finish", Point(0, 0))  # Default to (0, 0) if not found

    # Define the boundary based on corner points or other logic
    # Adjust the boundary definition as needed based on your requirements
    boundary_corners = [points.get(f"corner0{i}", Point(0, 0)) for i in range(1, 5)]
    boundary = Polygon(boundary_corners)

    return [start, finish, boundary]



if __name__ == "__main__":
    
    # checkpoints = ''
    # obstacles = ''
    # environment = ''
    
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
    
    # path_planner.checkpoints = compute_gate_midpoints(checkpoints)
    # path_planner.environment = parse_csv_for_points_and_boundary(environment)
    # path_planner.obstacles = read_obstacles(obstacles)
    
    #print("running path planner")
    #path_planner.global_prm()
    #print("finished path planner")
    
    while path_planner.index < path_planner.maxlen:
        
    
