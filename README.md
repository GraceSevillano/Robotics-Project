
# Autonomous Ground Robot by Perception

This repository contains the source code and documentation for a project that demonstrates an autonomous ground differential robot performing tasks through perception-driven control. The project is divided into two main parts: Autonomous Driving and Robot Collaboration Task.

## Part I: Autonomous Driving

The first part of the project focuses on the autonomous navigation of a ground differential robot using perception sensors.

### Objective
To enable a ground robot to navigate autonomously through a predefined path by leveraging vision sensors and encoders, without the use of LiDAR.

### Requirements
- Utilization of vision sensors and encoders only.
- Transition from the starting point to a delivery point to pick an object and return.
- Implementation of a state machine with clear, defined states.
- The return path should be different from the initial approach.
- Reception of a signal indicating the completion of the robotic arm's task (no timers are allowed).
- The delivery position is determined by detecting an ArUco tag.

### Implementation Details
- The TurtleBot Autorace 2020 platform was used as the base for this part.
- Intrinsic and extrinsic calibration parameters were fine-tuned.
- The `detect_lane` function from the `turtlebot3_autorace_detect` package was modified to improve lane detection accuracy by implementing smaller detection rectangles.
- Custom Python scripts were developed for ArUco detection and interaction with the Niryo robot arm, facilitating the subscription and publication to relevant ROS topics.

## Part II: Tag Recognition & VSLAM

In the second part of the project, the robot performs collaborative tasks using tag recognition and Visual Simultaneous Localization and Mapping (VSLAM).

### Objective
To demonstrate effective collaboration between ground robots using only visual perception for navigation and task execution.

### Running the Program

1. **TurtleBot3 Setup:**
   ```sh
   ssh ubuntu@192.168.0.200
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
   ```

2. **Niryo Robot Setup:**
   ```sh
   ssh niryo@192.168.0.150
   ```

3. **Remote PC Setup:**
   - Start ROS core:
     ```sh
     roscore
     ```
   - Launch camera calibration:
     ```sh
     roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
     roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
     ```
   - Start lane detection:
     ```sh
     roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
     ```
   - Run custom Python scripts:
     ```sh
     rosrun turtlebot3_autorace_driving detect_aruco.py
     rosrun turtlebot3_autorace_driving Niryo.py
     ```
   - Begin autonomous driving control:
     ```sh
     roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
     ```

## Videos


## Contributors

- Grace Sevillano
