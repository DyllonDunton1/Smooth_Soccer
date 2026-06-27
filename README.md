# Smooth Soccer

A ROS 2 mobile robotics project for planning and executing a JetBot soccer path using ArUco detections, RRT path planning, path smoothing, and open-loop movement commands.

The robot identifies tagged objects in a tabletop soccer scene, builds a simple robot-relative world model, plans a route around obstacles, smooths the route, and publishes velocity commands to drive the JetBot toward a ball and goal objective.

## Repository Contents

The main code is inside a ROS 2 workspace snapshot:

```text
Smooth_Soccer/
├── README.md
├── Dunton-Dyllon-ECE598-Final-Report.pdf
└── mobilerobotics_ws/
    ├── src/
    │   └── pathing/
    │       ├── package.xml
    │       ├── setup.py
    │       └── pathing/
    │           ├── better_soccer.py
    │           ├── standard_angle.py
    │           └── standard_movement.py
    ├── build/
    ├── install/
    └── log/
```

The important source code is in:

```text
mobilerobotics_ws/src/pathing/pathing/
```

The checked-in `build/`, `install/`, and `log/` folders are generated ROS/colcon artifacts from the original course workspace. They are not the main files to review. For understanding the project, start with the `pathing` package source files.

## ROS 2 Package

The ROS 2 package is:

```text
pathing
```

It is an `ament_python` package described in:

```text
mobilerobotics_ws/src/pathing/package.xml
```

The package exposes three console scripts in `setup.py`:

```python
planner = pathing.better_soccer:main
angle = pathing.standard_angle:main
forward = pathing.standard_movement:main
```

These correspond to:

| Script | File | Purpose |
|---|---|---|
| `planner` | `better_soccer.py` | Full ArUco-to-RRT-to-motion pipeline |
| `angle` | `standard_angle.py` | Helper for reading/printing ArUco-derived marker positions and angles |
| `forward` | `standard_movement.py` | Helper for tuning repeatable open-loop forward/turn movement |

## Main Planner

The main project file is:

```text
mobilerobotics_ws/src/pathing/pathing/better_soccer.py
```

This file contains the complete planning and motion pipeline.

It defines helper classes for:

```text
LandMark
Goal
Point
twoWheelBot
RRT
SmoothSoccer
```

The high-level flow is:

```text
ArUco detections
  -> landmark extraction
  -> obstacle / ball / goal model
  -> target approach pose
  -> RRT path search
  -> path smoothing
  -> turn / forward command sequence
  -> /jetbot/cmd_vel
```

## Inputs and Outputs

The planner subscribes to:

```text
aruco_detections
```

with message type:

```text
aruco_opencv_msgs/msg/ArucoDetection
```

The planner publishes to:

```text
/jetbot/cmd_vel
```

with message type:

```text
geometry_msgs/msg/Twist
```

The planner waits until it sees five markers, then assigns marker IDs to field objects:

```text
ID 0 -> goal reference
ID 4 -> second goal marker
ID 1 -> ball
ID 2 -> obstacle 1
ID 3 -> obstacle 2
```

These detections are converted into a simple robot-relative coordinate system before planning.

## RRT Path Planning

The `RRT` class builds a path from the robot origin to an approach position behind the ball.

The approach position is calculated from the ball-to-goal direction. Instead of driving directly to the ball, the robot approaches from an angle that lines the ball up with the goal.

The planner:

1. builds a bounding region around the robot, obstacles, ball, and goal
2. randomly samples points in that region
3. connects each sampled point to the nearest existing tree node
4. rejects points too close to obstacles
5. stops when a point is close enough to the calculated approach position
6. backtracks through parent nodes to recover the final path

## Path Smoothing

Raw RRT paths are often jagged and contain too many intermediate points for a small physical robot to follow cleanly.

After a path is found, `better_soccer.py` performs a smoothing pass. It checks whether later path nodes can connect directly to earlier nodes without passing too close to obstacles. If a direct segment is valid, intermediate nodes are removed.

This reduces the number of waypoints and creates a simpler command sequence with fewer turns.

That matters because the JetBot motion is open-loop. Every extra turn creates another opportunity for accumulated motion error.

## Motion Command Generation

After smoothing, the planner converts the path into travel stops and distances.

For each segment, it computes:

```text
target yaw
turn angle
forward distance
```

Then the `SmoothSoccer` node runs the sequence using:

```python
turn(angle_cw)
forward(distance)
reset()
```

The motion commands are published as `Twist` messages to `/jetbot/cmd_vel`.

Current tuning constants in `better_soccer.py` include:

```python
turn_speed = (3.0 / 4.0) * pi
forw_speed = 0.1
rot_mult = 0.32
lin_mult = 6.7
```

The forward command also applies a small angular correction to counteract drift during open-loop driving.

## Movement Calibration Helpers

Two helper scripts are included for calibrating and debugging movement.

### `standard_angle.py`

```text
mobilerobotics_ws/src/pathing/pathing/standard_angle.py
```

This node subscribes to `aruco_detections` and prints converted marker positions and yaw angles.

It was used to inspect the ArUco coordinate conversion and understand how the camera detections mapped into the robot-relative frame.

### `standard_movement.py`

```text
mobilerobotics_ws/src/pathing/pathing/standard_movement.py
```

This node publishes direct `/jetbot/cmd_vel` commands for repeatable turning and forward movement tests.

It was used to tune constants such as:

```python
turn_speed
forw_speed
rot_mult
lin_mult
```

Those constants helped make open-loop movement more predictable before integrating the full planner.

## Demo

### Example Map and Physical Layout

<img src="https://github.com/user-attachments/assets/f2ca4f31-774a-4d78-8489-e0ff9796ca02" width="400" height="280">

### Demo Solving the Problem

![robosoccer-demo](https://github.com/user-attachments/assets/e459ffc4-8905-4e08-a2d2-6bcd78da8941)

## How to Build

From the ROS 2 workspace:

```bash
cd mobilerobotics_ws
colcon build
source install/setup.bash
```

The original project used ROS 2 Foxy-era tooling and an ArUco detection package that published `aruco_opencv_msgs/msg/ArucoDetection`.

## How to Run

After building and sourcing the workspace, the main planner should be launched with:

```bash
ros2 run pathing planner
```

The helper scripts are:

```bash
ros2 run pathing angle
ros2 run pathing forward
```

The full planner expects an ArUco detection node to already be running and publishing marker detections on:

```text
aruco_detections
```

It also expects the JetBot command topic:

```text
/jetbot/cmd_vel
```

## My Contribution

My main contribution was the pathing module inside the `pathing` package.

I wrote:

- the RRT-style path planner in `better_soccer.py`
- the path smoothing logic
- the target approach-pose calculation
- the conversion from planned path to turn/forward commands
- the movement calibration helpers in `standard_angle.py` and `standard_movement.py`

The ArUco detection module itself was not my module. This repository focuses on the planning and motion side that consumes the ArUco detections.

## Technologies Used

- ROS 2
- Python
- rclpy
- JetBot-style mobile robot
- ArUco marker detections
- `aruco_opencv_msgs/msg/ArucoDetection`
- `geometry_msgs/msg/Twist`
- RRT path planning
- Path smoothing
- Matplotlib visualization
- Open-loop differential-drive motion

## Project Status

This is an archived course project repository. It is useful as a snapshot of the planning, smoothing, and movement-calibration work, but it is not organized as a clean reusable robotics package.

The best files to inspect first are:

```text
mobilerobotics_ws/src/pathing/pathing/better_soccer.py
mobilerobotics_ws/src/pathing/pathing/standard_angle.py
mobilerobotics_ws/src/pathing/pathing/standard_movement.py
mobilerobotics_ws/src/pathing/setup.py
```
