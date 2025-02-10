from collections import deque
from vexcode_vr import *

# ================================================================================
# =========================== VEXcode VR Initialisation ==========================
# ================================================================================
brain = Brain()
drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

# ================================================================================
# ============================== Constant setup ==================================
# ================================================================================
# Heading offsets: 0 = North, 1 = East, 2 = South, 3 = West.
OFFSETS = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}
CELL_SIZE = 250  # Distance (mm) between cells.
WALL_THRESHOLD = 260  # Minimum sensor reading (mm) to consider a corridor open.

# Symbols for maze visualisation
WALL = "🟥"  # Wall
OPEN = "🟩"  # Open cell/passage
ROUTE = "🟦"  # Fastest route
START_EXIT = "⬛"  # Start and exit marker


def get_neighbour_cell(position, direction):
    """
    Given a maze cell position and a heading (0=N, 1=E, 2=S, 3=W),
    return the coordinate of the neighbouring cell.
    """
    dx, dy = OFFSETS[direction]
    return position[0] + dx, position[1] + dy


# ================================================================================
# =========================== Maze Navigator Class ===============================
# ================================================================================
class MazeNavigator:
    def __init__(self):
        self.position = (0, 0)
        self.heading = 0
        self.graph = {}
        self.visited = set()
        self.exit_cell = None

    def add_connection(self, cell_a, cell_b):
        """
        Add a connection between two maze cells in the graph.
        """
        self.graph.setdefault(cell_a, set()).add(cell_b)
        self.graph.setdefault(cell_b, set()).add(cell_a)

    # ------------------ Movement functions ------------------
    def set_heading(self, target_heading):
        """
        Turn the robot to face the given heading (0=N, 1=E, 2=S, 3=W).
        """
        try:
            turn_amount = (target_heading - self.heading) % 4
            if turn_amount == 1:
                drivetrain.turn_for(RIGHT, 90, DEGREES)
            elif turn_amount == 2:
                drivetrain.turn_for(RIGHT, 180, DEGREES)
            elif turn_amount == 3:
                drivetrain.turn_for(LEFT, 90, DEGREES)
            self.heading = target_heading
        except Exception as e:
            print("An error occurred during heading adjustment:", e)

    def move_forward(self):
        """
        Move the robot forward by one cell and update its position.
        """
        try:
            drivetrain.drive_for(FORWARD, CELL_SIZE, MM)
            self.position = get_neighbour_cell(self.position, self.heading)
        except Exception as e:
            print("An error occurred during forward movement:", e)

    # ------------------ Detection Functions ------------------
    def is_path_clear(self):
        """
        Return True if the front-eye sensor detects no obstacle ahead.

        There is a special case for the start and ending cells as these have an open wall below and above respectively.
        The special case ensures that the robot does not leave the map.
        """
        try:
            if self.position == (0, 0) and self.heading == 2:
                return False
            if (
                self.exit_cell is not None
                and self.position == self.exit_cell
                and self.heading == 0
            ):
                return False
            return front_distance.get_distance(MM) >= WALL_THRESHOLD
        except Exception as e:
            print("An error occurred during path detection:", e)
            return False

    def is_at_exit(self):
        """
        Uses the eye sensor to detect the red exit marker.
        """
        try:
            return down_eye.detect(RED)
        except Exception as e:
            print("An error occurred during exit detection:", e)
            return False

    # ------------------ Depth First Search ------------------
    def explore_maze(self, current_cell, current_heading):
        """
        Recursively explore the maze using depth-first search (DFS), trying to minimize the number of turns.

        For each cell:
            1. Mark the cell as visited.
            2. Check if it is the exit and record it.
            3. Try moving in four relative directions in the following order:
                - Forward (offset 0)
                - Left (offset -1)
                - Right (offset 1)
                - Back (offset 2)
            4. If the path is clear, add the neighbour to the graph.
            5. If the neighbour is unvisited, move into it and recursively explore.
            6. After exploring, backtrack to the current cell.
        """
        try:
            # Mark the current cell as visited.
            self.visited.add(current_cell)

            # Check if the current cell is the exit, record the exit cell if found.
            if self.is_at_exit() and self.exit_cell is None:
                self.exit_cell = current_cell
                print(f"Exit detected at {current_cell}")

            # Try directions in order: forward, left, right, back.
            for offset in [0, -1, 1, 2]:
                new_heading = (current_heading + offset) % 4
                self.set_heading(new_heading)

                # If the path in the new heading is clear, process the neighbour.
                if self.is_path_clear():
                    neighbour = get_neighbour_cell(current_cell, new_heading)
                    self.add_connection(current_cell, neighbour)

                    # Only explore if the neighbour has not been visited.
                    if neighbour not in self.visited:
                        self.move_forward()
                        self.explore_maze(neighbour, new_heading)
                        # Backtrack turn around, move back, turn back to original heading.
                        self.set_heading((new_heading + 2) % 4)
                        self.move_forward()
                        self.set_heading(current_heading)
        except Exception as e:
            print("An error occurred during maze exploration:", e)

    # ------------------ Breadth First Search ------------------
    def find_shortest_path(self, start, goal):
        """
        Find the shortest path from the start to the goal using breadth-first search (BFS).
        Return the path as a list of cell coordinates.
        """
        try:
            # Use a dictionary to store the predecessors of each cell in the path.
            predecessors = {start: None}
            # Initialise the BFS queue with the start cell.
            queue = deque([start])
            while queue:
                current = queue.popleft()
                if current == goal:
                    break

                # Add unvisited neighbours to the queue to explore.
                for neighbour in self.graph.get(current, set()):
                    if neighbour not in predecessors:
                        predecessors[neighbour] = current
                        queue.append(neighbour)

            # Reconstruct the path from the goal to the start using the predecessors.
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = predecessors.get(node)
            return list(reversed(path))
        except Exception as e:
            print("An error occurred during pathfinding:", e)
            return []

    # ------------------ Path Traversal ------------------
    def traverse_path(self, path):
        """
        Traverse the given path by following the sequence of cell coordinates.
        For each pair of adjacent cells, turn towards the next cell and move forward.
        """
        direction_map = {(0, 1): 0, (1, 0): 1, (0, -1): 2, (-1, 0): 3}
        for i in range(1, len(path)):
            try:
                current, next_cell = path[i - 1], path[i]
                delta = (next_cell[0] - current[0], next_cell[1] - current[1])
                target_heading = direction_map.get(delta)
                if target_heading is None:
                    continue
                self.set_heading(target_heading)
                self.move_forward()
            except Exception as e:
                print(f"Error traversing from {current} to {next_cell}: {e}")

    def solve_maze(self):
        """
        Run the maze solving process:
            1. Explore the maze (DFS) to map the layout and locate the exit.
            2. Compute the fastest route (BFS) from the start to the exit.
            3. Traverse the route to reach the exit and then return to the start.
            4. Display the maze layout and the fastest route using Unicode symbols.
        """
        try:
            print("Mapping maze...")
            self.explore_maze(self.position, self.heading)
            if self.exit_cell is None:
                print("Exit not found during mapping!")
                return

            print("Mapping complete.")
            print("Maze graph:", self.graph)

            # Print the maze layout.
            print("Maze layout:")
            print(
                generate_unicode_maze_text(self.graph, self.visited, (0, 0), self.exit_cell)
            )

            # Compute and display the fastest route from start to exit.
            route = self.find_shortest_path((0, 0), self.exit_cell)
            print("Fastest route:", route)
            print("Maze with route overlay:")
            print(
                generate_unicode_maze_text(
                    self.graph, self.visited, (0, 0), self.exit_cell, route
                )
            )

            print("Following route to exit...")
            self.traverse_path(route)
            print("At exit. Returning to base...")
            self.traverse_path(list(reversed(route)))
            print("Mission complete!")
        except Exception as e:
            print("An error occurred during maze solving:", e)

# -------------------- Unicode Maze Generation --------------------
def generate_unicode_maze_text(graph, visited, start, exit_cell, route=None):
    """
    Convert the maze graph and visited cells into a Unicode representation for visualisation.
    The maze is displayed with walls, open passages, the
    fastest route, and start/exit markers.

    The maze is represented as a grid of Unicode characters:
    - 🟥: Wall
    - 🟩: Open cell/passage
    - 🟦: Fastest route
    - ⬛: Start and exit marker
    """
    try:
        if not visited:
            return ""

        # ------------------ Grid Generation ------------------
        # Find the minimum and maximum coordinates of the visited cells.
        # These will be used to determine the grid dimensions. (Allows for potentially future expansion and handling
        # of larger mazes)
        min_x = min(x for (x, _) in visited)
        max_x = max(x for (x, _) in visited)
        min_y = min(y for (_, y) in visited)
        max_y = max(y for (_, y) in visited)

        # Calculate the grid dimensions based on the range of visited cells.
        num_cols = max_x - min_x + 1
        num_rows = max_y - min_y + 1
        grid_width = num_cols * 2 + 1
        grid_height = num_rows * 2 + 1

        # Initialize the grid with walls.
        grid = [[WALL for _ in range(grid_width)] for _ in range(grid_height)]

        # ------------------ Maze to Grid Conversion ------------------
        def maze_to_grid(maze_cell):
            """
            Convert a maze cell coordinate to the corresponding grid cell coordinate.
            The grid cell is the center of the corresponding maze cell.
            """
            x, y = maze_cell
            return (max_y - y) * 2 + 1, (x - min_x) * 2 + 1

        # Mark the open cells that have been visited.
        for cell in visited:
            r, c = maze_to_grid(cell)
            grid[r][c] = OPEN

        # ------------------ Add Passages ------------------
        # Add passages between visited cells that are connected in the graph.
        for cell in visited:
            if cell in graph:
                for neighbour in graph[cell]:
                    if neighbour in visited:
                        # Convert the maze cell coordinates to grid coordinates.
                        r1, c1 = maze_to_grid(cell)
                        r2, c2 = maze_to_grid(neighbour)

                        # Calculate the passage cell between the two cells.
                        passage_r = (r1 + r2) // 2
                        passage_c = (c1 + c2) // 2
                        grid[passage_r][passage_c] = OPEN

        # ------------------ Add Route and Start/Exit ------------------
        # Mark the fastest route cells with the distinct symbol.
        if route:
            for i in range(len(route)):
                r, c = maze_to_grid(route[i])
                grid[r][c] = ROUTE
                if i < len(route) - 1:
                    r_next, c_next = maze_to_grid(route[i + 1])
                    mid_r = (r + r_next) // 2
                    mid_c = (c + c_next) // 2
                    grid[mid_r][mid_c] = ROUTE

        # Mark the start and exit cells.
        sr, sc = maze_to_grid(start)
        er, ec = maze_to_grid(exit_cell)
        grid[sr][sc] = START_EXIT
        grid[er][ec] = START_EXIT

        # Convert the grid to a string representation.
        return "\n".join("".join(row) for row in grid)
    except Exception as e:
        print("An error occurred during maze generation:", e)
        return ""


# ================================================================================
# =============================== Main Routine ===================================
# ================================================================================
def main():
    """
    Create an instance of the MazeNavigator class and solve the maze.
    Additionally, set the drivetrain velocities for faster traversal and
    set the pen down to monitor the robot's path.
    """
    try:
        drivetrain.set_drive_velocity(100, PERCENT)
        drivetrain.set_turn_velocity(100, PERCENT)
        pen.move(DOWN)
        MazeNavigator().solve_maze()
    except Exception as e:
        print("An error occurred during maze solving:", e)


# Vexcode VR thread to run the main routine.
vr_thread(main)
