# Research Track II - ASSIGNMENT

## Overview

This project implements a robot navigation stack in ROS2, using **actions**, **TF2** and **components**. The robot can be commanded to reach a target pose `(x, y, theta)` in the environment, with support for goal cancellation and preemption.

The architecture follows the **2 components + 1 separate node**:
- `move_robot_server`: responsible for the robot motion. It executes the action goal by publishing velocities on `/cmd_vel` and handles cancellation and preemption. It acts as a **TF Broadcaster** (publishing the robot's pose from odometry) and a **TF Listener** (computing the transform between the robot and the target goal).

- `move_robot_client`: receives the goal position by subscribing to the `send_command` topic and forwards it to the action server.

- `interface`: Python node that allows the user to set, cancel or abort a goal by publishing messages on the `send_command` topic.

| Node | Type | Language | Role |
|------|------|----------|------|
| `MoveRobotServerNode` | Component | C++ | Action server — moves the robot **(TF Broadcaster & Listener)** |
| `MoveRobotClientNode` | Component | C++ | Action client — forwards commands to server |
| `InterfaceNode` | Standalone node | Python | User interface — keyboard input |

---

## Custom Interfaces

### Action — `MoveRobot.action`
The action uses **x**, **y** and **theta** as *goal* variables.  It provides the current position of the robot as *feedback*, and return a *result* containing the final position and a status message (`SUCCESS`, `CANCELED` or `ABORT`).
```
# Goal
float64 goal_position_x
float64 goal_position_y
float64 goal_position_theta
---
# Result
float64 final_position_x
float64 final_position_y
float64 final_position_theta
string message
---
# Feedback
float64 current_position_x
float64 current_position_y
float64 current_position_theta
```

### Message — `SendUserCommand.msg`
Message is used to comunicate between interface and client, it implements the variable for the goal position **(x, y, theta)** and a boolean flags to send the cancel or abort messages.

```
float64 x
float64 y
float64 theta
bool cancel
bool shutdown
```

---

## Build and Run

### Prerequisites
- **OS**: Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco
- **Simulators**:
  - Gazebo Sim (physical simulation environment),
  - RViz2 (visualizer for sensor data and the `target_goal` frame)
- **Terminal**: `xterm` (required for the interface node).

To install dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 xterm
```

### Installation
Clone this repository into your workspace. Then, clone the required simulation environment inside the `src` folder of the newly created directory:
```bash
# Create and enter your workspace
mkdir -p ~/<your_workspace>
cd ~/<your_workspace>

# Clone this project
git clone https://github.com/Chiaera/Research_Track_II

# Enter the project's src directory to clone the simulation
cd Research_Track_II/src
git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors

# Go back to the workspace root to build
cd ~/<your_workspace>/Research_Track_II
colcon build
source install/setup.bash
```

## Package Structure
Before execute the file, make sure your workspace follows this structure:
```
├── build/
├── install/
├── log/
├── src
    ├── bme_gazebo_sensors/    #Simulation environment
    ├── robot_interfaces/      #Custom action and message definitions
    │   ├── action/MoveRobot.action
    │   └── msg/SendUserCommand.msg
    ├── pkg_cpp/                   #C++ components (server+client)
    │   ├── include/pkg_cpp/
    │   │   ├── move_robot_server.hpp
    │   │   └── move_robot_client.hpp
    │   └── src/
    │       ├── move_robot_server.cpp
    │       └── move_robot_client.cpp
    ├── pkg_py/                    #Python user interface node
    │   └── pkg_py/interface.py
    └── robot_bringup/             #Launch file and RViz config
        ├── launch/move_robot.launch.py
        └── config/start_configuration.rviz
```

## Execution
Launch the full simulation and interface window:
```bash
ros2 launch robot_bringup move_robot.launch.py
```
**Note:** This launch file automatically suppresses the default RViz configuration from the `bme_gazebo_sensors` package to load the custom `start_configuration.rviz` located in `robot_bringup/config`.

## Usage
Once launched, an xterm window will open. Follow the on-screen instructions to enter coordinates *(x,y,θ)*. You can **cancel** or **preempt** (send a new goal) at any time.
