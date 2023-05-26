# Astar Shortest Path Algorithm

## Description

The A* algorithm is a pathfinding method that is frequently used in artificial intelligence and robotics. It is frequently used to determine the shortest path between two points on a map.

The A* algorithm operates by keeping a prioritized list of possible nodes to be investigated. The algorithm expands the node with the lowest cost function value (the sum of the actual cost of reaching the node from the start node and a heuristic estimate of the remaining cost to reach the goal node) at each iteration.


The A* algorithm's cost function is defined as f(n) = g(n) + h(n), where g(n) represents the actual cost of reaching node n from the start node and h(n) represents a heuristic estimate of the remaining cost to reach the destination node. The heuristic function h(n) must be acceptable, which means that it should never overstate the real cost of getting to the goal node.

When the A* algorithm selects the goal node or when there are no more candidate nodes to examine, it terminates. By tracing back the path from the goal node to the start node using the parent pointers that were set during the search, the shortest path between the start node and the goal node may be rebuilt.

Overall, the A* algorithm is a powerful and efficient pathfinding method that is frequently used in robotics, video games, and other applications where the shortest path between two places is required.

The `a_star.py` file imports the AStar class and retrieves the produced path as well as the nodes that have been investigated. They are then utilized to display the algorithm's operation and the final path on the pygame window.

The Map has been scaled up for better visualization and the robot clearance and stride once entered also scaled up for matching the map scale.

## Project Partners

-   Tarun Trilokesh (UID: 118450766)
-   Harshal Shirsath (UID: 119247419)

## Requirements

-   Python 3.x
-   NumPy library
-   Time module
-   tkinter
-   pygame
-   math
-   heapq
-   PIL

## Installation

1.  Install Python 3.x on your computer.
2.  Install NumPy library using the following command:
    ```
    pip install numpy tk pygame
    ``` 
3.  The math, heapq, PIL and time module comes with Python’s standard utility module, so there is no need to install it externally.

## Usage

1.  Open the file `a_star.py` in any Python IDE or text editor.
2.  Run the code `a_star.py` and user is prompted by tkinter window to set all the parameters, select start and end node. 
3.  Click submit button on the prompt and wait for the Map to be updated with the clearance value (it will be seen as white boundary around the obstacles) (wait time 15 secs approx).
4.  If the start node and end node are same prompt displays to try again (rerun the code). If Path not found prompt displays to try again (rerun the code).
5.  The explored nodes are animated and the final path is plotted.
6.  The source code can also be cloned from github repositiory. Clone any one of the repositories below: 
    - git clone https://github.com/tarunreddyy/Astar-shortest-path-algorithm.git
7.  Navigate to the "Astar_Shortest_Path_Algorithm" or "" directory and follow the instructions.


## Output

The output of the program displays the animation and the final path on the pygame window. 