# Maze navigator and solver using VEXcode VR

This code contains a python based maze navigator for VEXcode VR. The solution maps the maze, find the quickest exit route, and then returns home.

## Features
- **Maze Mapping:**
Uses Depth First Search (DFS) to map the maze and all cells, this ensures a full and complete map of the map.
- **Quickest exit route:**
Makes use of Breadth First Search (BFS) to find the shortest path from the start to the exit. BFS is ideal because it searches level by level, this guarantees that the first time the exit is found, the path taken is the shortest.
- **Return Home:**
After the exit is reached, the robot will retrace the computed fastest path and return to the starting point.
- **Stay within the maze:**
Some of the walls in the maze are open (above the exit and below the start) and if the robot moves through them, it will fall off the maze. The code has edge cases implemented to ensure the robot stays within the maze.
- **Visual representation:**
To show the robot has fully mapped the maze once the mapping is complete a visual representation of the maze is printed to the console. This representation uses UNICODE coloured squares to make it easier to read.

## The Two Algorithms
### Depth First Search (DFS)
I chose to use DFS to map the maze because its ideal for this use case. DFS is a recursive algorithm that visits all the nodes of a graph (or in this case the maze) along a branch before backtracking. This ensures that the robot will visit every cell in the maze and map it completely (As the task requires).

### Breadth First Search (BFS)
I chose to use BFS to find the quickest exit route, this is because BFS searches level by level and guarantees that the first time the exit is found, the path taken is the shortest. This is ideal for this use case as the task requires the shortest path to be found and taken.


## Tasks Attempted

1. **Find the quickest Route Out of the Maze:**
- Implemented BFS to find the quickest route out of the maze.
2. **Map the Maze:**
- Implemented DFS to map the maze and all cells. Providing a visual representation of the maze once mapping is complete.
3. **Return Home:**
- After the exit is reach the code will reverse the fastest path and move the robot back to the starting point.
4. **Navigate corridors without straying:**
- The robot will not stray off the maze and will stay within the walls of the maze.

## Code Structure
- **MazeNavigator Class:**
Contains methods to explore the maze, find the shortest path to the exit, and then traverse that path.
- `explore_maze`: Uses DFS to map the maze and all cells.
- `find_shortest_path`: Uses BFS to find the shortest path to the exit.
- `traverse_path`: Moves the robot along the provided path.

- **Visualisations:**
The function `generate_unicode_maze_text` converts the maze graph and visited cells into a unicode string, marking any walls, open cells, the final route it will take, and using a special marker to show the exit and start points.

## Usage

1. **Setup:**
Load the code into VEXcode VR selecting the Dynamic wall maze playground.
2. **Run the code:**
Execute the script. The robot will:
- Map the maze.
- Find the shortest path to the exit.
- Traverse the path to the exit.
- Return to the starting point.

## Sources used
### Path finding research:
- https://www.graphable.ai/blog/pathfinding-algorithms/ - General Pathfinding Algorithms
- https://neo4j.com/docs/graph-data-science/current/algorithms/pathfinding/ - General Pathfinding Algorithms
- https://medium.com/omarelgabrys-blog/path-finding-algorithms-f65a8902eb40 - General Pathfinding Algorithms
### DFS research:
- https://neo4j.com/docs/graph-data-science/current/algorithms/dfs/ 
- https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
- https://medium.com/swlh/solving-mazes-with-depth-first-search-e315771317ae
### BFS research:
- https://neo4j.com/docs/graph-data-science/current/algorithms/bfs/
- https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
- https://medium.com/@luthfisauqi17_68455/artificial-intelligence-search-problem-solve-maze-using-breadth-first-search-bfs-algorithm-255139c6e1a3
### Visualisation research:
- https://stackoverflow.com/questions/63179536/convert-a-simple-mono-drawing-image-to-a-2d-text-array