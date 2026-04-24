# Research Track II - ASSIGNMENT

## Overview

This project implements a robot navigation stack in ROS2, using **actions**, **TF2** and **components**. The robot can be commanded to reach a target pose `(x, y, theta)` in the environment, with support for goal cancellation and preemption.

The architecture follows the **2 components + 1 separate node**:
- `move_robot_server`: responsible for the robot motion. It reads the initial `/odom` position from the evironment. It executes the action goal by publishing velocities on `/cmd_vel`, and handles cancellation and preemption. It acts as a **TF Broadcaster** (publishing the robot's pose from odometry) and a **TF Listener** (computing the transform between the robot and the target goal).
- `move_robot_client`: receives the goal position by subscribing to the `send_command` topic and forwards it to the action server.
- `interface`: Python node that allows the user to set, cancel or abort a goal by publishing messages on the `send_command` topic.

| Node | Type | Language | Role |
|------|------|----------|------|
| `MoveRobotServerNode` | Component | C++ | Action server — moves the robot **(TF Broadcaster & Listener)** |
| `MoveRobotClientNode` | Component | C++ | Action client — forwards commands to server |
| `InterfaceNode` | Standalone node | Python | User interface — keyboard input |

---

## Package Structure

```
src/
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
---

## Custom Interfaces

### Action — `MoveRobot.action`
The action used as a goal the variable **x**, **y** and **theta** and it implements the same variables as a feedback, to let the user know the current position of the robot, and as a result; in this case it adds a messages that return the `SUCCESS` status (or `CANCELED` or `ABORT`).
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
  - Gazebo Sim (to display the physical simulation and environment),
  - RViz2 (to show the robot's sensor data and a target_goal frame at the commanded position.)
- **Terminal**: `xterm` (to open the interface window)

To install dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-rviz2 xterm
```

### Installation
Clone this repository into your workspace:
```
# clone the repository of Research Track II course
cd ~/<your_workspace>
git clone https://github.com/Chiaera/Research_Track_II
```
Clone the required simulation environment:
```bash
# inside the `src` directory clone the simulation:
cd ~/<your_workspace>/src
git clone -b rt2 https://github.com/CarmineD8/bme_gazebo_sensors
```
Build and source:
```bash
cd ~/<your_workspace>
colcon build
source install/setup.bash
```
## Execution
Launch the full simulation and interface window:
```bash
ros2 launch robot_bringup move_robot.launch.py
```
**Note:** This launch file automatically suppresses the default RViz configuration from the simulation package to load the custom `start_configuration.rviz` located in `robot_bringup/config`.

## Usage
Once launched, an xterm window will open. Follow the on-screen instructions to enter coordinates *(x,y,θ)*. You can **cancel** or **preempt** (send a new goal) at any time.
