import cv2
import numpy as np
import heapq
import csv
import matplotlib.pyplot as plt

# -------------------- Config --------------------
IMG_PATH = "maze.png"
GRID = 255                   # <-- force to 255 x 255
START_MARGIN = 25            # default start/goal margin from corners
DIST_PENALTY_WEIGHT = 10.0   # weight for "stay away from walls"
# ------------------------------------------------

# ---------- Load & force to 255x255 ----------
img_gray = cv2.imread(IMG_PATH, cv2.IMREAD_GRAYSCALE)
if img_gray is None:
    raise FileNotFoundError(f"Could not read {IMG_PATH}")

orig_h, orig_w = img_gray.shape
img_gray = cv2.resize(img_gray, (GRID, GRID), interpolation=cv2.INTER_NEAREST)

# Threshold so white=free(1), black=wall(0)
_, maze = cv2.threshold(img_gray, 127, 1, cv2.THRESH_BINARY)  # 1=free, 0=wall

# ---------- Distance transform (clearance) ----------
dist_transform = cv2.distanceTransform((maze * 255).astype(np.uint8), cv2.DIST_L2, 5)
max_dt = dist_transform.max()
dist_transform = dist_transform / max_dt if max_dt > 0 else dist_transform

# ---------- Helpers ----------
def heuristic(a, b):
    # Manhattan (use Euclidean if you prefer: np.hypot(a[0]-b[0], a[1]-b[1]))
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def nearest_free_cell(maze, seed_rc, max_search=GRID):
    """Return nearest free (maze==1) to the seed (row,col)."""
    from collections import deque
    r0, c0 = seed_rc
    if maze[r0, c0] == 1:
        return (r0, c0)
    visited = np.zeros_like(maze, dtype=bool)
    q = deque([(r0, c0)])
    visited[r0, c0] = True
    rows, cols = maze.shape
    while q:
        r, c = q.popleft()
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr, nc]:
                if maze[nr, nc] == 1:
                    return (nr, nc)
                visited[nr, nc] = True
                q.append((nr, nc))
    raise RuntimeError("No free cell found in maze.")

def astar_with_margin(maze, dist_transform, start, goal):
    rows, cols = maze.shape
    open_set = [(heuristic(start, goal), 0.0, start, [])]
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        path = path + [current]
        visited.add(current)

        if current == goal:
            print("maze solved")
            return path

        r, c = current
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr, nc] == 1:
                # Penalty: closer to wall => higher cost
                penalty = (1.0 - dist_transform[nr, nc]) * DIST_PENALTY_WEIGHT
                new_cost = cost + 1.0 + penalty
                priority = new_cost + heuristic((nr, nc), goal)
                heapq.heappush(open_set, (priority, new_cost, (nr, nc), path))
    return None

# ---------- Start & goal on 255x255 grid ----------
# (row, col) = (y, x)
seed_start = (START_MARGIN, START_MARGIN)
seed_goal  = (GRID - START_MARGIN, GRID - START_MARGIN)

start = nearest_free_cell(maze, seed_start)
goal  = nearest_free_cell(maze, seed_goal)

# ---------- Run A* ----------
path = astar_with_margin(maze, dist_transform, start, goal)

# ---------- Visualize ----------
plt.figure(figsize=(8, 6))
plt.imshow(dist_transform, cmap='hot')
if path:
    rr, cc = zip(*path)          # rows (y), cols (x)
    plt.plot(cc, rr, color='cyan', linewidth=2, label="A* Path")
plt.plot(start[1], start[0], "go", label="Start")
plt.plot(goal[1],  goal[0],  "bo", label="Goal")
plt.title("A* Path with Distance Penalty (255×255)")
plt.legend()
plt.show()

plt.figure()
plt.imshow(maze, cmap='gray')
if path:
    plt.plot(cc, rr, color='red')
plt.plot(start[1], start[0], "go")
plt.plot(goal[1],  goal[0],  "bo")
plt.title("A* with Safety Margin (binary maze)")
plt.show()

# ---------- Save CSVs ----------
if path:
    # Coordinates on 255×255 grid, relative to start, in (x,y) order
    sx, sy = start[1], start[0]
    path_xy = [(c, r) for (r, c) in path]
    path_xy_rel = [(x - sx, y - sy) for (x, y) in path_xy]

    with open("coordinates_255.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(path_xy_rel)

    # Optional: map back to original image scale
    scale_x = orig_w / float(GRID)
    scale_y = orig_h / float(GRID)
    path_orig_xy = [(int(round(x * scale_x)), int(round(y * scale_y))) for (x, y) in path_xy]
    sx_o, sy_o = int(round(sx * scale_x)), int(round(sy * scale_y))
    path_orig_xy_rel = [(x - sx_o, y - sy_o) for (x, y) in path_orig_xy]

    with open("coordinates_original_scale.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        w.writerows(path_orig_xy_rel)

    print("Saved coordinates_255.csv and coordinates_original_scale.csv")
else:
    print("No path found.")
