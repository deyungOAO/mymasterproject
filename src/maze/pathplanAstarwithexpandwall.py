import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the maze PNG
img = cv2.imread("maze.png", cv2.IMREAD_GRAYSCALE)

# Threshold: walls=1, free=0
_, maze = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)

plt.imshow(maze, cmap='gray')
plt.title("Maze as Occupancy Grid")
plt.show()
start = (25,25)  # (row, col)
row, col = start  # pick a known wall pixel


goal = (maze.shape[0]-25, maze.shape[1]-25)
import heapq

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(maze, start, goal):
    rows, cols = maze.shape
    open_set = [(0+heuristic(start, goal), 0, start, [])]
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        path = path + [current]
        visited.add(current)

        if current == goal:
            return path

        r, c = current
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(-1,-1),(1,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr, nc] == 1:
                heapq.heappush(open_set, (cost+1+heuristic((nr,nc), goal),
                                          cost+1, (nr,nc), path))
    return None
path = astar(maze, start, goal)
#  print("path",path) path=(x1,y1)....to...(xfinal,yfinal)
# Plot result
plt.imshow(maze, cmap='gray')
if path:
    y, x = zip(*path)
    plt.plot(x, y, color='red')
plt.title("A* Path Planning")
plt.show()



# Inflate obstacles
kernel = np.ones((6, 6), np.uint8)  # 7-pixel clearance
inverted = 1 - maze
dilated = cv2.dilate(inverted, kernel, iterations=1)
inflated_maze = 1 - dilated

# Run A* on inflated maze
path = astar(inflated_maze, start, goal)

# Visualize
plt.imshow(maze, cmap='gray')
if path:
    y, x = zip(*path)
    plt.plot(x, y, color='red')
plt.plot(start[1], start[0], "go")
plt.plot(goal[1], goal[0], "bo")
plt.title("A* with Safety Margin")
plt.show()
