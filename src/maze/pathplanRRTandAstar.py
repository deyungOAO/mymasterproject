import cv2
import numpy as np
import random
import math

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def dist(n1, n2):
    return math.hypot(n1.x - n2.x, n1.y - n2.y)

def steer(from_node, to_node, step_size):
    angle = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + step_size * math.cos(angle))
    new_y = int(from_node.y + step_size * math.sin(angle))
    return Node(new_x, new_y, from_node)

def collision_free(node, parent, maze):
    if not (0 <= node.y < maze.shape[0] and 0 <= node.x < maze.shape[1]):
        return False
    steps = int(dist(node, parent))
    for i in range(steps + 1):
        x = int(parent.x + i * (node.x - parent.x) / max(1, steps))
        y = int(parent.y + i * (node.y - parent.y) / max(1, steps))
        if maze[y, x] == 0:  # 0 = wall
            return False
    return True

def extract_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def rrt_with_astar_bias(maze, start, goal, astar_path,
                        step_size=5, max_iter=50000, goal_thresh=100, bias_prob=0.5):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    nodes = [start_node]

    for _ in range(max_iter):
        # Bias: 50% pick a point along the A* path
        if random.random() < bias_prob and astar_path:
            rand_point = random.choice(astar_path)
            rand_node = Node(rand_point[0], rand_point[1])
        else:
            rand_node = Node(random.randint(0, maze.shape[1]-1),
                             random.randint(0, maze.shape[0]-1))

        # Nearest node
        nearest = min(nodes, key=lambda n: dist(n, rand_node))

        # Steer
        new_node = steer(nearest, rand_node, step_size)

        # Collision check
        if collision_free(new_node, nearest, maze):
            nodes.append(new_node)

            # Goal check
            if dist(new_node, goal_node) < goal_thresh:
                return extract_path(new_node), nodes

    return None, nodes
import cv2
import matplotlib.pyplot as plt
from pathplanAstarwithexpandwall import astar
# Assume you already computed astar_path as a list of (x, y)
# Example dummy path for testing if you don’t have A*


# Load maze
img = cv2.imread("maze.png", cv2.IMREAD_GRAYSCALE)
_, maze = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)  # 1=free, 0=wall

start = (25, 25)
goal  = (maze.shape[1]-25, maze.shape[0]-25)
astar_path = astar(maze, start, goal)  # A* result included
# Run RRT with bias
path, nodes = rrt_with_astar_bias(maze, start, goal, astar_path)

# Visualization
plt.imshow(maze, cmap="gray")

# Draw the A* path (dashed red)
if astar_path:
    ay, ax = zip(*astar_path)
    plt.plot(ax, ay, 'r--', linewidth=2, label="A* Path new")

# Draw RRT tree
for node in nodes:
    if node.parent:
        plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'y-', linewidth=0.5)

# Draw RRT final path
if path:
    xs, ys = zip(*path)
    plt.plot(xs, ys, 'g-', linewidth=2, label="RRT Path")

# Start and goal
plt.plot(start[0], start[1], "go", markersize=8)
plt.plot(goal[0], goal[1], "bo", markersize=8)
plt.legend()
plt.title("RRT with A* Biasing")
plt.show()
