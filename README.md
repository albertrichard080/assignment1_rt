First Assignment of Research Track
# Assignment 1 – Research Track I (ROS 2)

**Student:** Richard Albert K M  
**Course:** Research Track I  
**Instructor:** Carmine Tommaso Recchiuto  
**Student Number:** 8525970  

---

## Overview

This repository contains the solution for **Assignment 1** of the Research Track I course.
The assignment is implemented using **ROS 2 (Humble)** and **turtlesim**, and it consists of
a ROS 2 package named `assignment1_rt` with two nodes:

1. **UI Node** – allows the user to control the turtles via terminal input  
2. **Distance Node** – monitors the distance between turtles and enforces safety rules  

## Node 1 – UI Node

**File:** `ui_node.py`

### Description

The UI node provides a simple **text-based interface** using terminal input.
The user can:

- Select which turtle to control (`turtle1` or `turtle2`)
- Enter linear velocity (`x`) and angular velocity (`z`)
- Send the command for **1 second**
- Automatically stop the turtle after 1 second
- Repeat the process multiple times

This node is intentionally implemented as a **blocking UI**, since it is human-driven.

### Topics

- **Published**
  - `/turtle1/cmd_vel` (`geometry_msgs/Twist`)
  - `/turtle2/cmd_vel` (`geometry_msgs/Twist`)

---

## Node 2 – Distance Node

**File:** `distance_node.py`

### Description

The Distance node is responsible for **monitoring safety conditions**.
It continuously checks the relative distance between `turtle1` and `turtle2`
and ensures that unsafe motion is stopped.

The node performs the following tasks:

- Subscribes to the pose of both turtles
- Computes the **Euclidean distance** between them
- Publishes the distance on a dedicated topic
- Stops the turtles if:
  - The distance between them is below a threshold
  - A turtle approaches the simulation boundaries

### Topics

- **Subscribed**
  - `/turtle1/pose` (`turtlesim/Pose`)
  - `/turtle2/pose` (`turtlesim/Pose`)
- **Published**
  - `/distance` (`std_msgs/Float32`)
  - `/turtle1/cmd_vel` (`geometry_msgs/Twist`)
  - `/turtle2/cmd_vel` (`geometry_msgs/Twist`)

---

## Design Notes

- The UI node sends a **single velocity command**, waits for one second, and then sends a
  stop command. In turtlesim, the last velocity command remains active until a new one is received.
- The Distance node continuously monitors safety conditions using subscriber callbacks.
- If unsafe conditions are detected, the Distance node publishes zero velocity commands,
  which override any motion commands.
- This design ensures that **safety always has priority** over user commands.

---

## How to Run the Assignment

```bash

1. Source ROS 2
source /opt/ros/humble/setup.bash

2. Build the workspace
colcon build
source install/setup.bash

3. Start turtlesim
ros2 run turtlesim turtlesim_node

4. Spawn the second turtle
ros2 run assignment1_rt turtle_spawn

5. Run the Distance node
ros2 run assignment1_rt distance_node

6. Run the UI node
ros2 run assignment1_rt uinode 
