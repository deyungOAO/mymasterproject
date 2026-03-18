import random
import matplotlib.pyplot as plt
import cv2
import os

# Step 1: Generate Maze (0 = wall, 1 = free)
def generate_maze(width, height):
    assert width % 2 == 1 and height % 2 == 1, "Width/height must be odd"
    maze = [[0 for _ in range(width)] for _ in range(height)]
    start_x, start_y = 1, 1
    maze[start_y][start_x] = 1
    directions = [(0,2), (2,0), (0,-2), (-2,0)]

    def carve(x, y):
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 1 <= nx < width-1 and 1 <= ny < height-1 and maze[ny][nx] == 0:
                maze[ny][nx] = 1
                maze[y + dy//2][x + dx//2] = 1
                carve(nx, ny)

    carve(start_x, start_y)

    # ensure entrance & exit free
    maze[1][1] = 1
    maze[height-2][width-2] = 1
    return maze, (1, 1), (width-2, height-2)  # (y, x)

# Step 2: Export Maze to SDF (static boxes)
def maze_to_sdf(maze, block_size=1.0):
    sdf_template = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="maze">
    <static>true</static>
    {links}
  </model>
</sdf>
"""
    links = []
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 0:  # wall
                # IMPORTANT: x first, then y
                pose = f"{y*block_size} {x*block_size} {block_size/2} 0 0 0"
                link = f"""
    <link name="wall_{y}_{x}">
      <pose>{pose}</pose>
      <collision name="collision">
        <geometry>
          <box><size>{block_size} {block_size} {block_size}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{block_size} {block_size} {block_size}</size></box>
        </geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient></material>
      </visual>
    </link>"""
                links.append(link)
    return sdf_template.format(links="\n".join(links))

# Step 3: Save Maze as PNG (0 black = wall, 1 white = free)
def save_maze_png(maze, entrance, exit, filename="maze.png"):
    plt.figure(figsize=(6,6))
    # Use 'gray' so 0->black, 1->white (matches our convention)
    plt.imshow(maze, cmap="gray", interpolation="nearest")
    # Optional markers:
    # ey, ex = entrance
    # xy, xx = exit
    # plt.scatter(ex, ey, c="lime", s=60, marker="o", label="Entrance")
    # plt.scatter(xx, xy, c="red", s=60, marker="o", label="Exit")
    plt.axis("off")
    # Only call legend if you actually plotted markers
    # plt.legend(loc="upper right")
    plt.savefig(filename, bbox_inches="tight", pad_inches=0)
    plt.close()

# Optional: write a minimal world that includes the model file
def write_world_for_model(model_path, world_path="maze_world.sdf"):
    abs_model = os.path.abspath(model_path)
    world = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <gravity>0 0 -9.81</gravity>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Include the generated model by file path -->
    <include>
      <uri>file://{abs_model}</uri>
      <name>maze</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
    with open(world_path, "w") as f:
        f.write(world)
    return world_path

# Main
if __name__ == "__main__":
    width, height = 21, 21
    maze, entrance, exit = generate_maze(width, height)

    # 1) Save PNG first (so it exists if you want to post-process)
    save_maze_png(maze, entrance, exit, "maze.png")
    print("✅ Maze image saved to maze.png")

    # 2) (Optional) If your Nav2 YAML needs white=free, black=wall you’re already good.
    #    If your pipeline expects the opposite, invert and overwrite or save a new file:
    # img = cv2.imread("maze.png", cv2.IMREAD_GRAYSCALE)
    # inv = cv2.bitwise_not(img)
    # cv2.imwrite("maze_inverted.png", inv)
    # print("✅ Inverted maze saved to maze_inverted.png")

    # 3) Write the static box model SDF
    sdf_content = maze_to_sdf(maze, block_size=0.05)
    with open("maze_model.sdf", "w") as f:
        f.write(sdf_content)
    print("✅ Maze SDF model saved to maze_model.sdf")

    # 4) (Optional) Write a world file that includes the model for easy testing
    world_path = write_world_for_model("maze_model.sdf", "maze_world.sdf")
    print(f"✅ World file saved to {world_path} (run: gz sim {world_path})")
