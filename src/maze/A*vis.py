import cv2
import numpy as np
import matplotlib.pyplot as plt
import heapq
import time

# ---------- Step 1: Load Maze ----------
img = cv2.imread("maze.png", cv2.IMREAD_GRAYSCALE)

# Threshold so white=free (1), black=wall (0)
_, maze = cv2.threshold(img, 127, 1, cv2.THRESH_BINARY)

rows, cols = maze.shape

# ---------- Step 2: Heuristic Function ----------
def heuristic(a, b):
    # Manhattan distance
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

# ---------- Step 3: A* Search with Visualization ----------
def astar_visual(maze, start, goal, delay=0.05):
    open_set = [(heuristic(start, goal), 0, start, [])]
    visited = set()

    plt.ion()
    fig, ax = plt.subplots()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        path = path + [current]
        visited.add(current)

        # ---------- Visualization ----------
        ax.clear()
        ax.imshow(maze, cmap='gray')
        if path:
            y, x = zip(*path)
            ax.plot(x, y, color='red')  # explored path so far
        ax.plot(start[1], start[0], "go")  # start = green
        ax.plot(goal[1], goal[0], "bo")   # goal = blue
        ax.set_title("A* Expansion")
        plt.pause(delay)

        if current == goal:
            plt.ioff()
            return path

        r, c = current
        for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:  # 4 directions
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr, nc] == 1:
                heapq.heappush(open_set, (cost+1+heuristic((nr,nc), goal),
                                          cost+1, (nr,nc), path))

    plt.ioff()
    return None

# ---------- Step 4: Run the Visualization ----------
start = (30, 30)  # Adjust to a free cell
goal  = (rows-30, cols-30)  # Adjust to a free cell

path = astar_visual(maze, start, goal)

# ---------- Step 5: Show Final Path ----------
plt.imshow(maze, cmap='gray')
if path:
    y, x = zip(*path)
    plt.plot(x, y, color='red')
plt.plot(start[1], start[0], "go")
plt.plot(goal[1], goal[0], "bo")
plt.title("Final Path")
plt.show()
