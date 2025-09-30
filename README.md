# IDA* Path Planner (ROS 2)

This package implements an Iterative Deepening A* (IDA*) path planner for grid-based environments in ROS 2.  
It integrates with the ROS 2 Navigation Stack (Nav2) and works with occupancy grid maps provided by the Map Server.  
The planner computes a path from a given start pose to a goal pose while avoiding obstacles.

---

## Requirements

- **ROS 2 Humble** (or your ROS 2 distro)
- **colcon** build system
- **Nav2** stack
- **Rviz2** for visualization

---

## Workspace Setup

First, create a new ROS 2 workspace if you don’t already have one:

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## Clone this repository in the root of the workspace

```bash
git clone https://github.com/W-OK-E/IDAstar.git
```

## Building the package

Navigate to the root of the workspace and build:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ida_star_planner
```

Source the Workspace
```bash
source install/setup.bash
```

## Running the Planner

Launch the map:
```bash
ros2 launch path map_launch.py
```

Launch the IDA* planner node:
```bash
ros2 run cruise cruise
```

## Visualize in Rviz:
```bash
rviz2
```

#### You should see the robot, the occupancy grid map, and the planned path.

Package Structure
```bash
src/
├── cruise/    #Package responsible for path planning 
    |── src/                      # Source file for IDAstar                   
    ├── CMakeLists.txt          #Cmake config files
    └── package.xml             #Xml specifying package requirements and structure
├── path/                        # Package responsible for maps
    |── launch/                      # Example maps                   
    ├── maps/                      # Example maps
    ├── CMakeLists.txt          #Cmake config files
    └── package.xml         #Xml specifying package requirements and structure
```

## Notes

- Tested on ROS 2 Humble with Ubuntu 22.04.
- Example maps (.pgm + .yaml) are included in the maps/ directory.
- Tested on Unitree GO2(to be integrated).
