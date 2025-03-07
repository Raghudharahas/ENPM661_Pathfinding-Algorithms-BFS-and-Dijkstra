import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import time

# Function to get valid user input for start and goal positions
def get_valid_position(maze, prompt):
    while True:
        try:
            x, y = map(int, input(prompt).split())  # Accepts user input as "x y"
            if 0 <= x < maze.shape[0] and 0 <= y < maze.shape[1]:
                if maze[x, y] == 0:  # Ensure it's a valid free space
                    return np.array([x, y])
                else:
                    print("Position inside an obstacle! Try again.")
            else:
                print(" Position out of bounds! Try again.")
        except ValueError:
            print("Invalid input! Enter two space-separated integers.")


def bfs_maze(maze,start_pos,goal_pos):
    rows,cols = maze.shape
    directions=[(0,1),(1,0),(0,-1),(-1,0),(1,1),(-1,-1),(1,-1),(-1,1)]
    queue = deque()
    visited_pos = set()
    queue.append((start_pos),[tuple(start_pos)]) # (position, distance)
    visited_pos.add(tuple(start_pos))
    while queue:
        current_pos,path = queue.popleft()
        #goal check 
        if np.array_equal(current_pos == goal_pos):
            return path
        
        #to explore the neighbours
        for dx,dy in directions:
            new_pos=np.add(current_pos,(dx,dy))
            
            #check if the new position is within the maze
            if 0<=new_pos[0]< rows and 0<=new_pos[1]<cols:
                if maze[new_pos[0],new_pos[1]] == 0 and tuple(new_pos) not in visited_pos:
                    queue.append((new_pos,path+[tuple(new_pos)]))
                    visited_pos.add(tuple(new_pos)) #mark the position as visited
       
    return "Failure"

#defining the maze
maze=np.ones((50,180)) #creating a 50x180 maze
maze[5:45,5:175]=0 #creating a wall in the maze

maze[10:40,30:40]=1 #E
maze[10:40,45:55]=1 #N
maze[10:40,60:70]=1 #p
maze[10:40,75:85]=1 #M
maze[10:40,90:100]=1 #6
maze[10:40,105:115]=1 #6
maze[10:40,120:130]=1 #1

#Inflate obstacles with 2mm clearance
def inflate_obstacle(maze,clearance=2):
    inflated_maze = np.copy(maze)
    rows,cols = maze.shape
    for i in range(rows):
        for j in range(cols):
            if maze[i,j] == 1:
                for dx in range(-clearance,clearance+1):
                    for dy in range(-clearance,clearance+1):
                        new_pos = (i+dx,j+dy)
                        if 0<=new_pos[0]<rows and 0<=new_pos[1]<cols:
                            inflated_maze[new_pos[0],new_pos[1]]=1
    return inflated_maze

maze = inflate_obstacle(maze,clearance=2)

#define start and goal positions 
start_pos=np.array([5,5])
goal_pos=np.array([45,175])# Get valid start and goal positions from user
start_pos = get_valid_position(maze, "Enter START coordinates (x y): ")
goal_pos = get_valid_position(maze, "Enter GOAL coordinates (x y): ")


#Run BFS to find the path
start_time = time.time()
path = bfs_maze(maze, start_pos, goal_pos)
end_time = time.time()

# Visualize Path in the Grid
def plot_maze(maze, path, start_pos, goal_pos):
    plt.figure(figsize=(10, 5))
    
    # Plot Maze (0 = free space, 1 = obstacle)
    plt.imshow(maze, cmap="gray_r")  # Obstacles are black, free space is white

    # Plot the Path if BFS found one
    if path != "Failure" and path is not None:
        px, py = zip(*path)
        plt.plot(py, px, marker="o", color="red", markersize=3, linestyle="-", linewidth=1, label="Path")

    # Mark Start and Goal positions
    plt.scatter(start_pos[1], start_pos[0], color="green", marker="o", label="Start")  # Green = Start
    plt.scatter(goal_pos[1], goal_pos[0], color="blue", marker="o", label="Goal")  # Blue = Goal

    # Add Grid for Better Visualization
    plt.grid(True, linestyle="--", linewidth=0.5)
    
    # Set X and Y Labels
    plt.xticks(range(0, maze.shape[1], 10))  # Label every 10 units
    plt.yticks(range(0, maze.shape[0], 10))  # Label every 10 units
    plt.xlabel("X (Columns)")
    plt.ylabel("Y (Rows)")

    plt.legend()
    plt.title("BFS Path in the Maze")
    plt.show()

# Show the Path
plot_maze(maze, path, start_pos, goal_pos)

plot_maze(maze,path)
if path == "Failure":
    print("No path found")
else:
    print("Path found")
    
