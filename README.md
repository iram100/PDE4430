

##PDE4430- Mobile Robotics Coursework 

## Repository Link

**GitHub Repository:**
https://github.com/iram100/PDE4430

---

## Video Demonstration

**YouTube / Video Link:**
https://youtu.be/hzzkVm3rkjs?si=kqTC30YyV7yp6YfN

---

## Project Overview

This project presents the design and implementation of a **teleoperated mobile robot** capable of interacting with and moving spherical objects of different sizes into a designated **pen area** formed by walls in the provided `assessment_world`.

The robot is developed using **ROS 2 Jazzy** and simulated in **Gazebo**, following the constraints and requirements of the coursework specification.
The solution demonstrates a clear understanding of:

* Mobile robot kinematics
* Differential drive control
* Physical interaction with objects in simulation
* ROS 2 node, topic, and launch architecture

---

## Objectives

The main objectives of this coursework are:

1. Design a custom mobile robot using URDF/Xacro
2. Spawn and control the robot in the provided assessment world
3. Enable reliable teleoperation using ROS velocity commands
4. Demonstrate interaction with spherical objects of varying sizes
5. Successfully move the spheres into the pen area
6. Maintain clean project structure and version control using Git

---

## Approach

### Robot Design

The robot uses a **differential drive configuration** with two powered wheels and a rigid chassis.
The design was intentionally kept simple and robust to ensure reliable object interaction and stable motion during teleoperation.

Key design considerations:

* Increased robot mass to reduce sphere bounce
* Wider chassis to improve pushing stability
* Appropriately sized wheels for smooth ground contact
* Forward-facing geometry suitable for pushing objects

### Control Strategy

The robot is controlled using **keyboard teleoperation** via the `/cmd_vel` topic.
Velocity commands are bridged between ROS 2 and Gazebo using `ros_gz_bridge`, allowing real-time control of the robot inside the simulation.

The task is completed by manually navigating the robot to each sphere and pushing it toward the pen area.

---

## System Architecture

### ROS Nodes

* `robot_state_publisher` – Publishes robot transforms
* `ros_gz_bridge` – Bridges ROS 2 and Gazebo topics
* `teleop_twist_keyboard` – Sends velocity commands

### Topics

* `/cmd_vel` – Linear and angular velocity commands
* `/odom` – Odometry information
* `/tf` – Coordinate transforms

### Simulation

* **World:** `assessment_world`
* **Objects:** Small, medium, and large spheres with different masses
* **Physics:** Gazebo physics engine with tuned friction and inertia values

---

## Project Structure

```
ros2_ws/
├── src/
│   ├── assessment_world/        # Provided simulation world
│   ├── my_bot_description/      # Robot description and launch files
│   │   ├── urdf/
│   │   │   └── my_bot.urdf.xacro
│   │   └── launch/
│   │       └── bringup.launch.py
│   ├── my_bot_control/          # Control utilities (if used)
│   └── my_bot_teleop/           # Teleoperation helpers (if used)
```

---

## How to Build and Run

### Step 1 – Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Step 2 – Launch the Full System

A single launch file is provided to start the entire system.

```bash
ros2 launch my_bot_description bringup.launch.py
```

This launch file:

* Starts Gazebo with the assessment world
* Spawns the robot
* Starts required ROS nodes
* Bridges velocity commands

---

## Teleoperation

In a **new terminal**, run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r cmd_vel:=/cmd_vel
```

Use the keyboard to drive the robot and push the spheres into the pen.

---

## Demonstration Guidelines

In the video demonstration, the following is shown:

1. Launching the simulation using the provided launch file
2. Robot spawning correctly in the assessment world
3. Keyboard teleoperation of the robot
4. Interaction with small, medium, and large spheres
5. Successful movement of spheres into the pen area
6. Explanation of robot design and system architecture

---

## Performance and Limitations

* The robot reliably moves all sphere sizes when approached carefully
* Heavier chassis reduces excessive sphere movement
* Teleoperation provides full control but requires manual precision
* The system is structured to allow future extension to autonomous operation

---

## Version Control

This project was developed using Git with:

* Multiple commits over the project lifecycle
* Logical separation of development stages
* Clear commit messages describing changes

---

## Conclusion

This project demonstrates a complete mobile robotics pipeline from robot design to simulation and teleoperation.
The solution satisfies the coursework requirements while maintaining clarity, robustness, and extensibility.

---
Iram Mukri
M01092222
