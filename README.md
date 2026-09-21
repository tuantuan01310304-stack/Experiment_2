Experiment 2 — mechArm270 Fixed-Point Grasping

ROS 2 Humble based fixed-point pick-and-place experiment using Gazebo / Ignition simulation and a real Elephant Robotics mechArm270 robotic arm.

The project focuses on a complete grasping workflow, from robot initialization and point-to-point motion planning to gripper control, object placement, execution logging, and abnormal-condition testing.

Repository structure

Experiment_2/
├── simulation/
│   ├── code/
│   │   ├── final_grasp.launch.py
│   │   ├── final_grasp_task.py
│   │   ├── lws_transfer.launch.py
│   │   ├── lws_transfer_task.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── setup2.py
│   ├── config/
│   │   ├── final_grasp_params.yaml
│   │   ├── lws_transfer_params.yaml
│   │   └── lws_transfer_world.sdf
│   ├── logs/
│   ├── screenshots/
│   └── README.md
│
├── real_robot/
│   ├── code/
│   ├── config/
│   ├── logs/
│   ├── screenshots/
│   └── README.md
│
└── report/
    └── # LaTeX experiment report

Project overview

The experiment implements a fixed-point pick-and-place task for the mechArm270.

The basic task sequence is:

Home
  ↓
Approach Point A
  ↓
Descend
  ↓
Close Gripper
  ↓
Lift
  ↓
Move to Point B
  ↓
Descend
  ↓
Release Object
  ↓
Lift
  ↓
Return Home

The same task concept is evaluated in both simulation and on the real robot.

Simulation versions

The simulation/code/ directory contains two generations of the task implementation.

Initial version

lws_transfer_task.py
lws_transfer.launch.py
lws_transfer_params.yaml

This is the initial implementation developed during the experiment. It is retained as the development version so that the repository preserves the original implementation and debugging history.

It provides the basic fixed-point transfer workflow and can still be launched independently.

Final version

final_grasp_task.py
final_grasp.launch.py
final_grasp_params.yaml

This is the final implementation used for the final simulation workflow.

Compared with the initial implementation, the final version reorganizes the task into clearer functional components, including robot kinematics, target solving, motion control, simulation-object handling, and high-level task execution.

The final version keeps the same experimental objective and uses the same robot model and simulation environment, while providing a more structured implementation for the final experiment.

Why both versions are retained

Both versions are intentionally kept in the repository.

The initial version documents the development stage of the experiment, while the final version represents the finalized implementation. Keeping both versions makes the software evolution and experimental workflow easier to trace and reproduce.

The two task nodes are registered as separate ROS 2 executables, so one implementation does not overwrite the other.

Simulation workflow

The simulation uses:

ROS 2 Humble

Gazebo / Ignition Fortress

ros2_control

ROS 2 trajectory action interfaces

Robot state publishing

Gazebo–ROS communication through ros_gz_bridge

The launch process starts the simulation world, robot model, state publisher, controllers, and task node in sequence.

The simulation parameters define:

Robot base pose

Point A and Point B

Approach/safe height

Tool-tip offset

Gripper opening and closing positions

Arm and gripper motion duration

Number of repeated grasping cycles

Simulation checking options

Real-robot workflow

The real_robot/ directory contains the corresponding implementation and records for physical-arm experiments.

The real-robot experiment includes:

Robot driver startup

Joint-state feedback

Fixed-point grasping

Execution logs

Safety and abnormal-condition testing

The simulation is used to validate the task logic before transferring the workflow to the physical mechArm270.

Experimental evidence

Simulation scene



Simulation execution



Simulation terminal



Real-robot task execution



ROS 2 real-arm driver



Joint-state feedback



Safety / abnormal test



Validation

The real-robot validation record contains five successful runs out of five, with no collision recorded.

The repository also preserves trajectory records, execution logs, and an abnormal joint-limit test as supporting experimental evidence.

Reproducibility

The repository keeps the task source code, parameter files, simulation world, launch files, logs, screenshots, and experiment report together.

This organization allows the simulation and real-robot experiments to be reproduced from the recorded configuration rather than relying only on the final written report.

Platform

Ubuntu 22.04

ROS 2 Humble

Gazebo / Ignition Fortress

MoveIt 2

ros2_control

Jetson Orin

Elephant Robotics mechArm270

Notes

Demonstration videos are submitted separately according to the course requirements.

The LaTeX experiment report is maintained under report/.
