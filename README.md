# Experiment 3.5 — mechArm270 Fixed-Point Grasping

ROS 2 Humble based fixed-point grasping experiment using a mechArm 270, including Gazebo/Ignition simulation and real-robot verification.

## Project structure

- `simulation/` — simulation ROS 2 code, configuration, logs, screenshots, and report materials.
- `real_robot/` — real-robot ROS 2 driver, grasp task, parameters, and validation logs.
- `report/` — LaTeX experiment report and report figures (to be completed).

## Experiment workflow

Home → move above pick point A → descend → close gripper → lift → move to place point B → release → return Home.

The experiment uses fixed pick/place points and does not use visual localization.

## Main validation results

- Simulation: fixed-point grasping workflow and repeated-run validation.
- Real robot: 5/5 independent A→B grasping runs succeeded.
- Real robot: five consecutive bidirectional transfer segments completed.
- Safety test: an out-of-limit joint target was rejected and the trajectory action returned `ABORTED` without robot motion.

## Platform

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Ignition + MoveIt 2 / ros2_control
- Jetson Orin
- Elephant Robotics mechArm 270-Pi

## Notes

The real-robot stage retains the ROS 2 trajectory-control architecture while changing hardware communication and real workspace parameters. Demonstration videos are submitted separately as required by the course.
