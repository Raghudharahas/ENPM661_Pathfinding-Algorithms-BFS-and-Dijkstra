import numpy as np
import cv2
import time
import heapq  # For priority queue

# Define Maze Dimensions (50 x 200)
maze = np.ones((50, 200))  
maze[5:45, 5:195] = 0  

# Define Obstacles (Letters ENPM661)
maze[10:40, 20:30] = 1   
maze[10:40, 35:45] = 1   
maze[10:40, 50:60] = 1   
maze[10:40, 65:75] = 1   
maze[10:40, 90:100] = 1  
maze[10:40, 115:125] = 1 
maze[10:40, 140:150] = 1 

# Inflate Obstacles by 2mm Clearance
def inflate_obstacle(maze, clearance=2):
    inflated_maze = np.copy(maze)
    rows, cols = maze.shape
    for i in range(rows):
        for j in range(cols):
            if maze[i, j] == 1:
                for dx in range(-clearance, clearance + 1):
                    for dy in range(-clearance, clearance + 1):
                        ni, nj = i + dx, j + dy
                        if 0 <= ni < rows and 0 <= nj < cols:
                            inflated_maze[ni, nj] = 1  
    return inflated_maze

maze = inflate_obstacle(maze, clearance=2)

# Function to Get Valid Start/Goal Positions
def get_valid_position(maze, prompt):
    while True:
        try:
            x, y = map(int, input(prompt).split())  
            if 0 <= x < maze.shape[0] and 0 <= y < maze.shape[1]:
                if maze[x, y] == 0:  
                    return np.array([x, y])
                else:
                    print(" Position inside an obstacle! Try again.")
            else:
                print("Position out of bounds! Try again.")
        except ValueError:
            print("Invalid input! Enter two space-separated integers.")

# ✅ Dijkstra's Algorithm Implementation
def dijkstra_maze(maze, start_pos, goal_pos):
    rows, cols = maze.shape
    directions = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
    
    OpenList = []  # Priority queue
    ClosedList = set()  
    parent_map = {}  
    CostToCome = {tuple(start_pos): 0}  

    heapq.heappush(OpenList, (0, tuple(start_pos)))  
    parent_map[tuple(start_pos)] = None  

    while OpenList:
        cost, current_pos = heapq.heappop(OpenList)  
        ClosedList.add(current_pos)  

        # If goal is reached, backtrack the path
        if current_pos == tuple(goal_pos):
            return backtrack_path(parent_map, goal_pos), ClosedList  

        #  Explore neighbors
        for dx, dy in directions:
            new_pos = (current_pos[0] + dx, current_pos[1] + dy)

            if 0 <= new_pos[0] < rows and 0 <= new_pos[1] < cols:
                if maze[new_pos[0], new_pos[1]] == 0 and new_pos not in ClosedList:
                    step_cost = 1.4 if dx != 0 and dy != 0 else 1  
                    new_cost = CostToCome[current_pos] + step_cost  

                    if new_pos not in CostToCome or new_cost < CostToCome[new_pos]:
                        CostToCome[new_pos] = new_cost  
                        heapq.heappush(OpenList, (new_cost, new_pos))  
                        parent_map[new_pos] = current_pos  

    return "Failure", ClosedList  

#  Function to Backtrack from Goal to Start Using Parent Tracking
def backtrack_path(parent_map, goal_pos):
    path = []
    node = tuple(goal_pos)  

    while node is not None:
        path.append(node)
        node = parent_map[node]  

    return path[::-1]  

#  Get Valid Start & Goal Positions
start_pos = get_valid_position(maze, "Enter START coordinates (x y): ")
goal_pos = get_valid_position(maze, "Enter GOAL coordinates (x y): ")

#  Run Dijkstra and Measure Runtime
start_time = time.time()
path, explored_nodes = dijkstra_maze(maze, start_pos, goal_pos)
end_time = time.time()

#  Save Animation as Video
video_filename = "dijkstra_enpm661.mp4"
frame_size = (maze.shape[1] * 10, maze.shape[0] * 10)  
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(video_filename, fourcc, 10, frame_size)

#  Create Blank Image to Draw On
canvas = np.ones((maze.shape[0] * 10, maze.shape[1] * 10, 3), dtype=np.uint8) * 255

#  Draw Obstacles
for i in range(maze.shape[0]):
    for j in range(maze.shape[1]):
        if maze[i, j] == 1:
            cv2.rectangle(canvas, (j * 10, i * 10), (j * 10 + 10, i * 10 + 10), (0, 0, 0), -1)

#  Animate Dijkstra’s Exploration
for x, y in explored_nodes:
    cv2.rectangle(canvas, (y * 10, x * 10), (y * 10 + 10, x * 10 + 10), (255, 0, 0), -1)
    out.write(canvas)

#  Draw Final Path if Found
if path != "Failure":
    for x, y in path:
        cv2.rectangle(canvas, (y * 10, x * 10), (y * 10 + 10, x * 10 + 10), (0, 0, 255), -1)
        out.write(canvas)

#  Draw Start & Goal Positions
cv2.rectangle(canvas, (start_pos[1] * 10, start_pos[0] * 10), (start_pos[1] * 10 + 10, start_pos[0] * 10 + 10), (0, 255, 0), -1)
cv2.rectangle(canvas, (goal_pos[1] * 10, goal_pos[0] * 10), (goal_pos[1] * 10 + 10, goal_pos[0] * 10 + 10), (255, 0, 255), -1)
out.write(canvas)

#  Release Video Writer
out.release()

#  Print Results
print(f" Dijkstra Animation Saved as: {video_filename}")
print(f" Dijkstra Execution Time: {end_time - start_time:.4f} seconds")
if path == "Failure":
    print(" No path found")
else:
    print(" Path found:", path)
