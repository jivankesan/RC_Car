import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from shapely.geometry import Point, LineString, Polygon

# Parameters
NUM_SAMPLES = 400
NEIGHBOR_RADIUS = 10

# Obstacles with a bounding radius, represented as buffered points
obstacles = [
    Point(5.4, 5.34).buffer(0.8),  # garyTheSnail
    Point(5, 15).buffer(1),        # theStrip1
    Point(8, 15).buffer(1),        # theStrip2
    Point(8, 15).buffer(1),        # theStrip3 (assuming a typo in the image, adjusted for uniqueness)
    Point(8, 15).buffer(1),        # theStrip4 (assuming a typo in the image, adjusted for uniqueness)
    Point(8, 6).buffer(1),         # theStrip5
    Point(8, 6).buffer(1),         # theStrip6 (assuming a typo in the image, adjusted for uniqueness)
    Point(-8.654, 20.6).buffer(0.5),  # tunnel1
    Point(-7.89, 19).buffer(0.5),     # tunnel2
    Point(-8.675, 19.6).buffer(0.5),  # tunnel3
    # Add more obstacles if needed
]
checkpoints = [
    Point(4, -0.5),  # gate01
    Point(4, 1.5),    # gate02
    Point(4.8, 3.8), # gate03 (assumed coordinate, adjust as necessary)
    Point(5.6, 3.8),   # finishRamp
    Point(10.85, 24.96),
    # Add more checkpoints if needed
]
start = Point(0, 0)  # Example start point
finish = Point(23.126, 28.968)  # Example finish point

# Define the boundary of the environment (example values)
boundary = Polygon([(0, 0), (0, 30), (30, 30), (30, 0)])

# Function to check if a point is in the free space
def is_free(x, y):
    point = Point(x, y)
    return not any(point.within(obstacle) for obstacle in obstacles) and point.within(boundary)

# Function to check if a path between two points is free of obstacles
def is_path_free(p1, p2):
    line = LineString([p1, p2])
    return not any(line.intersects(obstacle) for obstacle in obstacles)

# Sample points
samples = []
while len(samples) < NUM_SAMPLES:
    x, y = np.random.uniform(0, 30), np.random.uniform(0, 30)
    if is_free(x, y):
        samples.append((x, y))

# Ensure the start, checkpoints, and finish are in the samples
important_points = [start] + checkpoints + [finish]
for point in important_points:
    samples.append((point.x, point.y))

# Create a KD-tree for efficient nearest neighbor search
tree = KDTree(samples)

# Build the graph
graph = nx.Graph()
for sample in samples:
    neighbors = tree.query_ball_point(sample, NEIGHBOR_RADIUS)
    for neighbor_idx in neighbors:
        neighbor = samples[neighbor_idx]
        if is_path_free(sample, neighbor):
            graph.add_edge(sample, neighbor, weight=np.linalg.norm(np.array(sample) - np.array(neighbor)))

# Find the shortest path through the checkpoints
try:
    path = [tuple(start.coords)[0]]
    for checkpoint in checkpoints:
        path_segment = nx.shortest_path(graph, path[-1], tuple(checkpoint.coords)[0], weight='weight')
        path.extend(path_segment[1:])  # Exclude the first point to avoid duplication
    path.extend(nx.shortest_path(graph, path[-1], tuple(finish.coords)[0], weight='weight')[1:])
except nx.NetworkXNoPath as e:
    print(f"No path could be found: {e}")
    path = []

for p in path:
    print(path)

# Plotting
fig, ax = plt.subplots()

# Plot the environment boundary
x, y = boundary.exterior.xy
ax.plot(x, y, 'b')

# Plot obstacles
for obstacle in obstacles:
    x, y = obstacle.exterior.xy
    ax.fill(x, y, 'r')

# Plot checkpoints
for checkpoint in checkpoints:
    plt.plot(checkpoint.x, checkpoint.y, 'yo')

# Plot start and finish
plt.plot(start.x, start.y, 'go')
plt.plot(finish.x, finish.y, 'mo')

# Plot the PRM graph
for (node1, node2) in graph.edges():
    x1, y1 = node1
    x2, y2 = node2
    plt.plot([x1, x2], [y1, y2], 'k-', lw=0.5)

# Plot the shortest path if found
if path:
    x_path, y_path = zip(*path)
    ax.plot(x_path, y_path, 'c-', lw=2, label='Optimal Path')

# Show plot
plt.axis('equal')
plt.legend()
plt.show()
