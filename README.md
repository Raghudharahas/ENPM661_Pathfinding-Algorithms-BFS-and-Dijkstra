# ENPM661_project2# Pathfinding Algorithms: BFS and Dijkstra 

## Overview
This project implements two pathfinding algorithms, **Breadth-First Search (BFS)** and **Dijkstra's Algorithm**, to navigate a maze containing obstacles. The goal is to find an optimal path from a **start position** to a **goal position** while avoiding obstacles and their clearance areas.

## Dependencies
Ensure you have the following Python libraries installed before running the code:
1. numpy 
2. opencv-python(cv2)
3. time
4. heapq(used only while running the Dijkstra's algorithm)

If dependencies are not installed then install them using : pip install numpy opencv-python


## How to Run the Code
1. **Run the script** in your Visual studio code:
    a:Download files and open in vs code and run
    b:Clone the repository to local system and run
 
2. **Enter start and goal coordinates** when prompted:
   - Input format: `row col` (e.g., `24 8`)
   - The program ensures the points are valid (not inside obstacles or inflated regions).
3. **Watch the visualization**:
   - The program visualizes the maze, explored nodes, and the final path.
   - A video file (`bfs_enpm661.mp4` or `Dijkstra_enpm661.mp4`) is generated.

## Input
- Start and goal coordinates in the format `row col`.
- The grid is represented as a **50x220** matrix where:
  - `0`: Free space
  - `1`: Obstacles
  - Blue border: Inflated obstacle regions (clearance included)

## Output
- Visualization of explored nodes and the optimal path.
- Execution time and number of explored nodes displayed.
- A recorded animation of the process (`.mp4` file).

## Algorithms Used
- **Breadth-First Search (BFS)**: Explores all nodes at the current depth level before moving deeper.
- **Dijkstra's Algorithm**: Uses a priority queue to explore the lowest-cost path first.

## Notes
- Press any key to proceed at different visualization stages.
- Press `ESC` to exit early during animations.

Regads,
Raghu dharahas

