# ENPM661_Project2 — Pathfinding Algorithms: BFS and Dijkstra

## 📌 Overview
This project implements two pathfinding algorithms, **Breadth-First Search (BFS)** and **Dijkstra's Algorithm**, to navigate a maze containing obstacles. The goal is to find an optimal path from a **start position** to a **goal position** while avoiding obstacles and their clearance areas.

---

## 📦 Dependencies

Ensure you have the following Python libraries installed:

- `numpy`  
- `opencv-python` (`cv2`)  
- `time` (standard library)  
- `heapq` *(used only for Dijkstra)*

📥 Install with:
```bash
pip install numpy opencv-python
```

---

## ▶️ How to Run the Code

1. **Run the script** in your preferred IDE (e.g., VS Code):
   - a. Download the `.py` files and run them
   - b. Or clone the repository and run locally

2. **Input Start and Goal Coordinates**:
   - Format: `row col` (e.g., `24 8`)
   - Validations prevent selecting points inside obstacles or inflated regions.

3. **Watch the Visualization**:
   - Maze, explored nodes, and the final path are animated.
   - A video file is generated for each algorithm:
     - `bfs_enpm661.mp4`
     - `Dijkstra_enpm661.mp4`

---

## 🧠 Input Details

- Start and goal inputs: `row col`
- Maze Grid: **50x220**
  - `0`: Free space  
  - `1`: Obstacle  
  - 🔵 Blue: Inflated obstacle border (for clearance)

---

## 🎯 Output

- Animated visualization of explored path and optimal route.
- Summary of:
  - Execution time  
  - Number of explored nodes  
- Video saved as `.mp4` (one per algorithm)

---

## 📊 Algorithms Implemented

- **Breadth-First Search (BFS)**  
  Explores all nodes at the current depth before going deeper. Good for shortest path in uniform cost grids.

- **Dijkstra’s Algorithm**  
  Uses a priority queue to always explore the lowest-cost node. Optimal for non-uniform cost problems.

---

## 🎥 Video Demos

- 🔴 **BFS Pathfinding Demo**  
  [![Watch BFS Video](https://img.youtube.com/vi/N4MKxKje930/hqdefault.jpg)](https://youtu.be/N4MKxKje930)

- 🟢 **Dijkstra Pathfinding Demo**  
  [![Watch Dijkstra Video](https://img.youtube.com/vi/0qdLtY-j7xM/hqdefault.jpg)](https://youtu.be/0qdLtY-j7xM?si=lPdnRAnafGrwNZYk)

---

## 📝 Notes

- Press any key during visualizations to proceed to the next stage.
- Press `ESC` to exit early during animations.
