import numpy as np
import cv2
import time
import heapq
from collections import deque

# Create a white background image for the maze
maze = np.ones((50, 220, 3), dtype=np.uint8) * 255

# Draw shape E
points_E = np.array([
    [20, 10], [33, 10], [33, 15], [25, 15], [25, 20], 
    [33, 20], [33, 25], [25, 25], [25, 30], [33, 30], 
    [33, 35], [20, 35], [20, 10]
])
# Fill the polygon with black color
cv2.fillPoly(maze, [points_E], (0, 0, 0), 1)

# Draw shape N
points_N = np.array([
    [45, 10], [50, 10], [55, 22], [55, 10], [60, 10],
    [60, 35], [55, 35], [50, 20], [50, 35], [45, 35], [45, 10]
])
cv2.fillPoly(maze, [points_N], (0, 0, 0), 1)

# Drawing shape P
points_P = np.array([
    [72, 10], [77, 10], [77, 10], [72, 10]
])
cv2.fillPoly(maze, [points_P], (0, 0, 0), 1)
cv2.rectangle(maze, (72, 10), (77, 35), (0, 0, 0), -1)

# Draw the semicircle for P
center = (77, 16)
radius = 6
cv2.ellipse(maze, center, (radius, radius), 0, -90, 90, (0, 0, 0), -1)

# Draw shape M
points_M = np.array([
    [95, 10], [100, 10], [105, 30], [110, 30], [115, 10],
    [120, 10], [120, 35], [115, 35], [115, 25], [110, 35],
    [105, 35], [100, 25], [100, 35], [95, 35], [95, 10]
])
cv2.fillPoly(maze, [points_M], (0, 0, 0), 1)

# Draw first 6
cv2.circle(maze, (143, 26), 9, (0, 0, 0), -1)
cv2.ellipse(maze, (143, 26), (10, 21), 0, -180, -75, (0, 0, 0), -1)

# Draw second 6
cv2.circle(maze, (173, 26), 9, (0, 0, 0), -1)
cv2.ellipse(maze, (173, 26), (10, 21), 0, -180, -75, (0, 0, 0), -1)

# Draw shape 1
cv2.rectangle(maze, (194, 10), (200, 35), (0, 0, 0), -1)

# Convert the RGB maze to binary for path planning
def convert_to_binary(maze_rgb):
    # Convert to grayscale
    maze_gray = cv2.cvtColor(maze_rgb, cv2.COLOR_BGR2GRAY)
    # Convert to binary where obstacles are 1 and free space is 0
    _, maze_binary = cv2.threshold(maze_gray, 128, 1, cv2.THRESH_BINARY_INV)
    return maze_binary

# Create an inflated obstacle map with different color for the inflated border
def create_inflated_map(maze_rgb, clearance=2):
    # Convert to binary
    maze_binary = convert_to_binary(maze_rgb)
    inflated_map = np.ones((maze_binary.shape[0], maze_binary.shape[1], 3), dtype=np.uint8) * 255
    
    # Mark original obstacles in black
    inflated_map[maze_binary == 1] = [0, 0, 0]  # Black for original obstacles
    
    # Create inflated binary map
    kernel = np.ones((2 * clearance + 1, 2 * clearance + 1), np.uint8)
    inflated_binary = cv2.dilate(maze_binary.astype(np.uint8), kernel, iterations=1)
    
    # Create a mask for just the inflated border (inflated minus original)
    border_mask = (inflated_binary == 1) & (maze_binary == 0)
    
    # Mark inflated borders in a different color (blue)
    inflated_map[border_mask] = [255, 0, 0]  # Blue for inflation border
    
    # Return both the visualization map and the binary inflated map for planning
    return inflated_map, inflated_binary

# Function to get valid start/goal positions
def get_valid_position(maze_binary, prompt):
    # Get start and goal positions from the user
    while True:
        try:
            x, y = map(int, input(prompt).split())
            if 0 <= x < maze_binary.shape[0] and 0 <= y < maze_binary.shape[1]:
                if maze_binary[x, y] == 0:
                    return np.array([x, y])
                else:
                    print("Position inside an obstacle or inflated area! Try again.")
            else:
                print("Position out of bounds! Try again.")
        except ValueError:
            print("Invalid input! Enter two space-separated integers.")

# Dijkstra's Algorithm Implementation
def dijkstra_maze(maze, start_pos, goal_pos):
    rows, cols = maze.shape
    directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
    
    OpenList = []  # Priority queue
    ClosedList = set()  # Visited nodes
    parent_map = {}  # For backtracking the path
    CostToCome = {tuple(start_pos): 0}  
    # Add the start position to the OpenList
    heapq.heappush(OpenList, (0, tuple(start_pos)))  
    parent_map[tuple(start_pos)] = None  
    # Run Dijkstra's algorithm
    while OpenList:
        cost, current_pos = heapq.heappop(OpenList)  
        
        # Skip if already in closed list
        if current_pos in ClosedList:
            continue
            
        ClosedList.add(current_pos)  

        # If goal is reached, backtrack the path
        if current_pos == tuple(goal_pos):
            return backtrack_path(parent_map, goal_pos), ClosedList  

        # Explore neighbors
        for dx, dy in directions:
            new_pos = (current_pos[0] + dx, current_pos[1] + dy)
            # Check if the new position is within bounds and is free
            if 0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols:
                if maze[new_pos[0], new_pos[1]] == 0 and new_pos not in ClosedList:
                    step_cost = 1.4 if dx != 0 and dy != 0 else 1  
                    new_cost = CostToCome[current_pos] + step_cost  
                    # If the new cost is less than the cost to come to the new position
                    if new_pos not in CostToCome or new_cost < CostToCome[new_pos]:
                        CostToCome[new_pos] = new_cost  
                        heapq.heappush(OpenList, (new_cost, new_pos))  
                        parent_map[new_pos] = current_pos  

    return "Failure", ClosedList  

# Function to Backtrack from Goal to Start Using Parent Tracking
def backtrack_path(parent_map, goal_pos):
    path = []
    node = tuple(goal_pos)  
    
    # Backtrack from goal to start
    while node is not None:
        path.append(node)
        node = parent_map[node]  
    # Reverse the path to get it from start to goal
    return path[::-1]  


# Create both the visualization map and the binary map for planning
inflated_maze_vis, inflated_maze_binary = create_inflated_map(maze, clearance=2)

# Display the original maze
resized_maze = cv2.resize(maze, (maze.shape[1] * 5, maze.shape[0] * 5), interpolation=cv2.INTER_NEAREST)
cv2.imshow('Original Maze', resized_maze)
print("Press any key to continue to display the inflated maze")
cv2.waitKey(0)
cv2.destroyAllWindows()
# Display the inflated maze with colored border
resized_inflated = cv2.resize(inflated_maze_vis, (inflated_maze_vis.shape[1] * 5, inflated_maze_vis.shape[0] * 5), interpolation=cv2.INTER_NEAREST)
cv2.imshow('Inflated Maze with Colored Border', resized_inflated)
print("Press any key to continue to enter start and goal positions")
cv2.waitKey(0)
cv2.destroyAllWindows()

# Get valid start and goal positions
print("Note: Enter coordinates as 'row col' (vertical, horizontal)")
start_pos = get_valid_position(inflated_maze_binary, "Enter START coordinates (x y): ")
goal_pos = get_valid_position(inflated_maze_binary, "Enter GOAL coordinates (x y): ")
    
# Run Dijkstra's algorithm
print("Running Dijkstra's algorithm...")
start_time = time.time()
path, explored_nodes = dijkstra_maze(inflated_maze_binary, start_pos, goal_pos)
end_time = time.time()
    
# Create a list to store frames
frames = []
# Create a canvas for visualization
exploration_viz = cv2.resize(inflated_maze_vis.copy(), (inflated_maze_vis.shape[1] * 5, inflated_maze_vis.shape[0] * 5), 
                           interpolation=cv2.INTER_NEAREST)

# Mark start and goal positions on the visualization
sx, sy = start_pos
gx, gy = goal_pos
cv2.rectangle(exploration_viz, (sy * 5, sx * 5), (sy * 5 + 5, sx * 5 + 5), (0, 0, 255), -1)  # Red for start
cv2.rectangle(exploration_viz, (gy * 5, gx * 5), (gy * 5 + 5, gx * 5 + 5), (255, 0, 255), -1)  # Magenta for goal
frames.append(exploration_viz.copy())

# Animate Dijkstra exploration frame by frame
for node in explored_nodes:
    x, y = node
    # Skip drawing if the node is start or goal
    if tuple(start_pos) == node or tuple(goal_pos) == node:
        continue
    cv2.rectangle(exploration_viz, (y * 5, x * 5), (y * 5 + 5, x * 5 + 5), (173, 216, 230), -1)  # Light blue for explored
    frames.append(exploration_viz.copy())
    
    # Show exploration frame-by-frame
    cv2.imshow('Exploration', exploration_viz)
    if cv2.waitKey(30) & 0xFF == 27:  # Press ESC to exit early
        break

# Draw final path if found
if len(path) > 0:
    for node in path:
        x, y = node
        # Skip drawing if the node is start or goal
        if tuple(start_pos) == node or tuple(goal_pos) == node:
            continue
        cv2.rectangle(exploration_viz, (y * 5, x * 5), (y * 5 + 5, x * 5 + 5), (0, 255, 0), -1)  # Green for path
        frames.append(exploration_viz.copy())
        
        # Show path drawing frame-by-frame
        cv2.imshow('Exploration', exploration_viz)
        if cv2.waitKey(30) & 0xFF == 27:  # Slower for path visualization
            break

# Wait for a key press before closing the window
cv2.waitKey(0)
cv2.destroyAllWindows()

# Create a video of the BFS exploration and path
output_video = "Dijkstra_enpm661.mp4"
frame_size = (inflated_maze_binary.shape[1] * 5, inflated_maze_binary.shape[0] * 5)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video, fourcc, 30, frame_size)

# Create a canvas for animation (use the inflated_maze_vis as base)
canvas = cv2.resize(inflated_maze_vis, (inflated_maze_vis.shape[1] * 5, inflated_maze_vis.shape[0] * 5), 
                   interpolation=cv2.INTER_NEAREST)

# Copy canvas for exploration animation
canvas_copy = canvas.copy()
# Mark start and goal positions on the canvas
for node in explored_nodes:
    x, y = node
    # Skip drawing if the node is start or goal
    if tuple(start_pos) == node or tuple(goal_pos) == node:
        continue
    cv2.rectangle(canvas_copy, (y * 5, x * 5), (y * 5 + 5, x * 5 + 5), (173, 216, 230), -1)  # Light blue for explored
    out.write(canvas_copy)

# Draw final path if found
if len(path) > 0:
    for node in path:
        x, y = node
        # Skip drawing if the node is start or goal
        if tuple(start_pos) == node or tuple(goal_pos) == node:
            continue
        cv2.rectangle(canvas_copy, (y * 5, x * 5), (y * 5 + 5, x * 5 + 5), (0, 255, 0), -1)  # Green for path
        out.write(canvas_copy)

# Mark start and goal positions
sx, sy = start_pos
gx, gy = goal_pos
cv2.rectangle(canvas_copy, (sy * 5, sx * 5), (sy * 5 + 5, sx * 5 + 5), (0, 0, 255), -1)  # Red for start
cv2.rectangle(canvas_copy, (gy * 5, gx * 5), (gy * 5 + 5, gx * 5 + 5), (255, 0, 255), -1)  # Magenta for goal
# Add a few more frames with the complete solution
for _ in range(30):  # Add 30 frames (1 second) of the final result
    out.write(canvas_copy)
# Release the video writer
out.release()

# Print results
print(f"Dijkstra Animation Saved as: {output_video}")
print(f"Dijkstra Execution Time: {end_time - start_time:.4f} seconds")
print(f"Nodes Explored: {len(explored_nodes)}")

# Print path length if found
if path == "Failure":
    print("No path found")
else:
    print(f"Path found with {len(path)} steps") # Path length
