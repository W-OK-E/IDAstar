#!/usr/bin/env python3
"""
generate_mazes_ros2.py

Generates random maze occupancy maps (PGM) and ROS2 YAML files for map_server.
Includes IDA* pathfinding and dynamic obstacle mode.

By default it creates 1 maze of size 100x100 cells:
  python generate_mazes_ros2.py

Options:
  --count N        Number of random maps to generate (default: 1)
  --size S         Width and height in cells (default: 100)
  --resolution R   Map resolution in meters per cell (default: 0.05)
  --outdir DIR     Output directory (default: ./maps)
  --dynamic        Enable dynamic obstacle mode with replanning
  --num_obstacles N  Number of dynamic obstacles to add (default: 5)
  --visualize      Show visualization of pathfinding
"""

import argparse
import os
import random
import numpy as np
import cv2
import time

class Node:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0
    
    def __repr__(self):
        return f"Node({self.x}, {self.y})"

class IDAStarPlanner:
    FOUND = -1
    INF = int(1e9)
    
    def __init__(self, grid, visualize=False, window_name="IDA* Planning", scale=8):
        """
        grid: 2D numpy array where 0 = free, 1 = wall
        scale: pixels per grid cell (default: 8, increase for larger windows)
        """
        self.grid = grid.copy()
        self.height, self.width = grid.shape
        self.visited = None
        self.path = []
        self.start = None
        self.goal = None
        self.explored_nodes = []
        self.visualize = visualize
        self.window_name = window_name
        self.obstacles = []
        self.scale = scale
        
        # Create resizable window if visualizing
        if self.visualize:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            # Set initial window size (width, height in pixels)
            cv2.resizeWindow(self.window_name, 800, 800)
        
        # 8-directional movement
        self.directions = [
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]
    
    def visualize_step(self, current_node=None):
        """Visualize current state of search"""
        if not self.visualize:
            return
        
        h, w = self.grid.shape
        img = np.zeros((h * self.scale, w * self.scale, 3), dtype=np.uint8)
        
        # Draw maze
        for y in range(h):
            for x in range(w):
                color = (255, 255, 255) if self.grid[y][x] == 0 else (0, 0, 0)
                cv2.rectangle(img, (x*self.scale, y*self.scale), 
                            ((x+1)*self.scale, (y+1)*self.scale), color, -1)
        
        # Draw explored nodes (blue)
        for x, y in self.explored_nodes:
            cv2.rectangle(img, (x*self.scale, y*self.scale), 
                        ((x+1)*self.scale, (y+1)*self.scale), (255, 200, 0), -1)
        
        # Draw dynamic obstacles (red)
        for x, y in self.obstacles:
            cv2.rectangle(img, (x*self.scale, y*self.scale), 
                        ((x+1)*self.scale, (y+1)*self.scale), (0, 0, 255), -1)
        
        # Draw current path being explored (light green)
        if self.path:
            for i in range(len(self.path) - 1):
                x1, y1 = self.path[i]
                x2, y2 = self.path[i + 1]
                cv2.line(img, (x1*self.scale + self.scale//2, y1*self.scale + self.scale//2),
                        (x2*self.scale + self.scale//2, y2*self.scale + self.scale//2), 
                        (100, 255, 100), 1)
        
        # Draw current node being explored (yellow)
        if current_node:
            x, y = current_node.x, current_node.y
            cv2.circle(img, (x*self.scale + self.scale//2, y*self.scale + self.scale//2), 
                      self.scale//2, (0, 255, 255), -1)
        
        # Draw start (cyan)
        if self.start:
            x, y = self.start.x, self.start.y
            cv2.circle(img, (x*self.scale + self.scale//2, y*self.scale + self.scale//2), 
                      self.scale, (255, 255, 0), -1)
        
        # Draw goal (magenta)
        if self.goal:
            x, y = self.goal.x, self.goal.y
            cv2.circle(img, (x*self.scale + self.scale//2, y*self.scale + self.scale//2), 
                      self.scale, (255, 0, 255), -1)
        
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)  # 1ms delay for smooth animation
    
    def h_cost(self, node):
        """Manhattan distance heuristic"""
        return abs(node.x - self.goal.x) + abs(node.y - self.goal.y)
    
    def dfs(self, curr, threshold):
        """DFS search with threshold"""
        f = curr.g_cost + self.h_cost(curr)
        
        if f > threshold:
            return f
        
        # Visualize current exploration
        self.visualize_step(curr)
        
        if curr.x == self.goal.x and curr.y == self.goal.y:
            self.path.append((curr.x, curr.y))
            return self.FOUND
        
        min_cost = self.INF
        self.visited[curr.y][curr.x] = True
        self.path.append((curr.x, curr.y))
        self.explored_nodes.append((curr.x, curr.y))
        
        for dx, dy in self.directions:
            new_x = curr.x + dx
            new_y = curr.y + dy
            
            if new_x < 0 or new_x >= self.width or new_y < 0 or new_y >= self.height:
                continue
            if self.grid[new_y][new_x] == 1:  # obstacle
                continue
            if self.visited[new_y][new_x]:
                continue
            
            neighbor = Node(new_x, new_y)
            # Diagonal moves cost sqrt(2), straight moves cost 1
            move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
            neighbor.g_cost = curr.g_cost + move_cost
            
            t = self.dfs(neighbor, threshold)
            if t == self.FOUND:
                return self.FOUND
            if t < min_cost:
                min_cost = t
        
        self.path.pop()
        self.visited[curr.y][curr.x] = False
        return min_cost
    
    def plan(self, start, goal):
        """Run IDA* algorithm"""
        self.start = start
        self.goal = goal
        self.visited = [[False] * self.width for _ in range(self.height)]
        self.path = []
        self.explored_nodes = []
        
        start.g_cost = 0
        start.h_cost = self.h_cost(start)
        start.f_cost = start.g_cost + start.h_cost
        
        threshold = start.f_cost
        
        while True:
            temp = self.dfs(start, threshold)
            if temp == self.FOUND:
                return self.path
            if temp == self.INF:
                return []  # No path found
            threshold = temp
    
    def update_grid(self, grid):
        """Update grid with new obstacles"""
        self.grid = grid.copy()
    
    def set_obstacles(self, obstacles):
        """Set obstacles for visualization"""
        self.obstacles = obstacles

def make_maze_grid(h, w, seed=None):
    """
    Generate a perfect maze on a grid of size (h x w) where h and w are integers.
    Approach:
      - Treat grid indices with both coordinates odd (1,3,5,...) as "cells".
      - Start with all walls (1). Carve passages (0) between odd indices using DFS.
    This works for even or odd h,w as long as there is at least one odd index inside the grid.
    Returns numpy array of dtype uint8: 0 -> free, 1 -> wall
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    grid = np.ones((h, w), dtype=np.uint8)  # 1 = wall, 0 = free

    # valid cell coordinates (odd indices inside the grid)
    cell_coords = [(r, c) for r in range(1, h, 2) for c in range(1, w, 2)]
    if not cell_coords:
        raise ValueError("Grid too small for maze carving; increase size.")

    # DFS stack
    start = random.choice(cell_coords)
    stack = [start]
    grid[start] = 0  # carve start cell

    # neighbor moves in cell steps (2 units in display space)
    moves = [(-2, 0), (2, 0), (0, -2), (0, 2)]

    while stack:
        cur = stack[-1]
        r, c = cur
        # shuffle moves
        random.shuffle(moves)
        carved = False
        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            # check bounds
            if 0 < nr < h and 0 < nc < w and grid[nr, nc] == 1:
                # carve through the wall between (r,c) and (nr,nc)
                wall_r, wall_c = r + dr // 2, c + dc // 2
                grid[wall_r, wall_c] = 0
                grid[nr, nc] = 0
                stack.append((nr, nc))
                carved = True
                break
        if not carved:
            stack.pop()
    return grid

def find_free_cells(grid):
    """Find all free cells in the grid"""
    free_cells = []
    h, w = grid.shape
    for y in range(h):
        for x in range(w):
            if grid[y][x] == 0:
                free_cells.append((x, y))
    return free_cells

def visualize_path(grid, path, explored_nodes, obstacles=None, save_path=None, scale=8):
    """Visualize the maze with path and explored nodes using OpenCV"""
    h, w = grid.shape
    # Create RGB image (scale up for better visibility)
    img = np.zeros((h * scale, w * scale, 3), dtype=np.uint8)
    
    # Draw maze
    for y in range(h):
        for x in range(w):
            color = (255, 255, 255) if grid[y][x] == 0 else (0, 0, 0)
            cv2.rectangle(img, (x*scale, y*scale), ((x+1)*scale, (y+1)*scale), color, -1)
    
    # Draw explored nodes (blue)
    for x, y in explored_nodes:
        cv2.rectangle(img, (x*scale, y*scale), ((x+1)*scale, (y+1)*scale), (255, 200, 0), -1)
    
    # Draw dynamic obstacles (red)
    if obstacles:
        for x, y in obstacles:
            cv2.rectangle(img, (x*scale, y*scale), ((x+1)*scale, (y+1)*scale), (0, 0, 255), -1)
    
    # Draw path (green)
    if path:
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            cv2.line(img, (x1*scale + scale//2, y1*scale + scale//2),
                    (x2*scale + scale//2, y2*scale + scale//2), (0, 255, 0), 2)
        
        # Draw start (cyan)
        x, y = path[0]
        cv2.circle(img, (x*scale + scale//2, y*scale + scale//2), scale, (255, 255, 0), -1)
        
        # Draw goal (magenta)
        x, y = path[-1]
        cv2.circle(img, (x*scale + scale//2, y*scale + scale//2), scale, (255, 0, 255), -1)
    
    if save_path:
        cv2.imwrite(save_path, img)
    
    return img

def save_pgm(filename, grid):
    """
    Save grid (0 free, 1 wall) as PGM file with ROS-friendly convention:
      - occupied (walls) -> 0 (black)
      - free -> 254 (almost white)
    PGM used is binary P5 with maxval 255.
    """
    h, w = grid.shape
    # Convert: walls (1) -> 0, free (0) -> 254
    img = np.where(grid == 1, 0, 254).astype(np.uint8)
    with open(filename, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())

def write_yaml(filename_yaml, image_filename, resolution=0.05, origin=(0.0, 0.0, 0.0),
               negate=0, occupied_thresh=0.65, free_thresh=0.196):
    """
    Write YAML content expected by ROS map_server / nav2.
    """
    yaml_text = f"""image: {image_filename}
resolution: {resolution}
origin: [{origin[0]}, {origin[1]}, {origin[2]}]
negate: {negate}
occupied_thresh: {occupied_thresh}
free_thresh: {free_thresh}
"""
    with open(filename_yaml, "w") as f:
        f.write(yaml_text)

def main():
    parser = argparse.ArgumentParser(description="Generate random mazes and ROS2 YAML maps with IDA* pathfinding.")
    parser.add_argument("--count", type=int, default=1, help="Number of maps to generate.")
    parser.add_argument("--size", type=int, default=100, help="Map width and height in cells (int).")
    parser.add_argument("--resolution", type=float, default=0.05, help="Meters per cell.")
    parser.add_argument("--outdir", type=str, default="maps", help="Output directory.")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed.")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic obstacle mode.")
    parser.add_argument("--num_obstacles", type=int, default=5, help="Number of dynamic obstacles.")
    parser.add_argument("--visualize", action="store_true", help="Show visualization.")
    parser.add_argument("--scale", type=int, default=8, help="Pixels per grid cell (default: 8, increase for larger window).")
    parser.add_argument("--window_size", type=int, default=800, help="Initial window size in pixels (default: 800).")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    for i in range(args.count):
        seed = args.seed if args.seed is not None else random.randrange(2**30)
        # create maze grid (0 free, 1 wall)
        grid = make_maze_grid(args.size, args.size, seed=seed)

        print(f"\n{'='*60}")
        print(f"Map {i+1}/{args.count} (seed={seed})")
        print(f"{'='*60}")

        # Find free cells for start and goal
        free_cells = find_free_cells(grid)
        if len(free_cells) < 2:
            print("Error: Not enough free cells for start and goal!")
            continue
        
        # Select start and goal
        start_coords = random.choice(free_cells)
        goal_coords = random.choice([c for c in free_cells if c != start_coords])
        
        start_node = Node(start_coords[0], start_coords[1])
        goal_node = Node(goal_coords[0], goal_coords[1])
        
        print(f"Start: {start_coords}, Goal: {goal_coords}")
        
        # Create planner with visualization flag
        planner = IDAStarPlanner(grid, visualize=args.visualize, 
                                window_name=f"Map {i+1} - IDA* Planning",
                                scale=args.scale)
        
        # Set window size if visualizing
        if args.visualize:
            cv2.namedWindow(planner.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(planner.window_name, args.window_size, args.window_size)
        
        if not args.dynamic:
            # Standard mode: solve once
            print("Running IDA* pathfinding...")
            start_time = time.time()
            path = planner.plan(start_node, goal_node)
            end_time = time.time()
            
            if path:
                print(f"✓ Path found! Length: {len(path)} nodes")
                print(f"  Time: {(end_time - start_time)*1000:.2f} ms")
                print(f"  Explored nodes: {len(planner.explored_nodes)}")
            else:
                print("✗ No path found!")
            
            if args.visualize:
                img = visualize_path(grid, path, planner.explored_nodes,
                                   save_path=os.path.join(args.outdir, f"map_{i+1}_path.png"),
                                   scale=args.scale)
                cv2.imshow(f"Map {i+1} - Final Result", img)
                cv2.resizeWindow(f"Map {i+1} - Final Result", args.window_size, args.window_size)
                print("\nPress any key to continue...")
                cv2.waitKey(0)
        
        else:
            # Dynamic mode: add obstacles and replan
            print("Running DYNAMIC mode with replanning...")
            current_grid = grid.copy()
            obstacles = []
            all_explored = []
            replan_count = 0
            
            # Initial plan
            print(f"\nInitial planning...")
            path = planner.plan(start_node, goal_node)
            if not path:
                print("✗ No initial path found!")
                continue
            print(f"✓ Initial path found! Length: {len(path)}")
            all_explored.extend(planner.explored_nodes)
            
            # Show final result after initial planning
            if args.visualize:
                img = visualize_path(current_grid, path, planner.explored_nodes, obstacles,
                                   save_path=os.path.join(args.outdir, f"map_{i+1}_initial.png"),
                                   scale=args.scale)
                cv2.imshow(f"Map {i+1} - Initial Path", img)
                cv2.resizeWindow(f"Map {i+1} - Initial Path", args.window_size, args.window_size)
                print("Initial path complete. Press any key to start adding obstacles...")
                cv2.waitKey(0)
            
            # Add dynamic obstacles and replan
            for obs_num in range(args.num_obstacles):
                # Find free cells not on current path and not start/goal
                path_set = set(path)
                available_cells = [c for c in free_cells 
                                 if c not in path_set 
                                 and c != start_coords 
                                 and c != goal_coords
                                 and c not in obstacles]
                
                if not available_cells:
                    print(f"No more space for obstacles!")
                    break
                
                # Add random obstacle
                obs_coords = random.choice(available_cells)
                obstacles.append(obs_coords)
                current_grid[obs_coords[1]][obs_coords[0]] = 1
                
                print(f"\n[Obstacle {obs_num+1}] Added at {obs_coords}")
                
                # Check if obstacle blocks current path
                if obs_coords in path_set:
                    print("  ⚠ Obstacle blocks path! Replanning...")
                    replan_count += 1
                    
                    # Update planner and replan
                    planner.update_grid(current_grid)
                    planner.set_obstacles(obstacles)
                    start_time = time.time()
                    path = planner.plan(start_node, goal_node)
                    end_time = time.time()
                    
                    if path:
                        print(f"  ✓ New path found! Length: {len(path)}")
                        print(f"    Time: {(end_time - start_time)*1000:.2f} ms")
                        print(f"    Explored nodes: {len(planner.explored_nodes)}")
                        all_explored.extend(planner.explored_nodes)
                    else:
                        print("  ✗ No path exists!")
                        break
                    
                    if args.visualize:
                        img = visualize_path(current_grid, path, planner.explored_nodes, obstacles,
                                           save_path=os.path.join(args.outdir, f"map_{i+1}_replan_{replan_count}.png"),
                                           scale=args.scale)
                        cv2.imshow(f"Map {i+1} - Replanning Result", img)
                        cv2.resizeWindow(f"Map {i+1} - Replanning Result", args.window_size, args.window_size)
                        print(f"  Replanning complete. Press any key to continue...")
                        cv2.waitKey(0)
                else:
                    print("  Path still valid, no replanning needed.")
            
            print(f"\n{'='*60}")
            print(f"Dynamic mode summary:")
            print(f"  Total obstacles added: {len(obstacles)}")
            print(f"  Replanning events: {replan_count}")
            print(f"  Total explored nodes: {len(all_explored)}")
            print(f"{'='*60}")
            
            if args.visualize:
                # Final visualization
                img = visualize_path(current_grid, path, all_explored, obstacles,
                                   save_path=os.path.join(args.outdir, f"map_{i+1}_final.png"),
                                   scale=args.scale)
                cv2.imshow(f"Map {i+1} - Final State", img)
                cv2.resizeWindow(f"Map {i+1} - Final State", args.window_size, args.window_size)
                print("\nFinal state. Press any key to continue...")
                cv2.waitKey(0)

        # Save original map files
        pgm_name = f"map_{i+1}.pgm"
        yaml_name = f"map_{i+1}.yaml"
        pgm_path = os.path.join(args.outdir, pgm_name)
        yaml_path = os.path.join(args.outdir, yaml_name)

        save_pgm(pgm_path, grid)
        write_yaml(yaml_path, pgm_name, resolution=args.resolution)

        print(f"\nSaved: {pgm_path}")
        print(f"Saved: {yaml_path}")
    
    if args.visualize:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()