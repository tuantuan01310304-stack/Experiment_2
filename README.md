# Experiment 2 — mechArm270 Fixed-Point Grasping

ROS 2 Humble based fixed-point pick-and-place experiment using Gazebo / Ignition simulation and a real mechArm270 robotic arm.

The project focuses on a complete grasping workflow, from robot initialization and point-to-point motion planning to gripper control, object placement, execution logging, and abnormal-condition testing.

## Project overview

The experiment implements a fixed-point pick-and-place task for the mechArm270.

The basic task sequence is:

```text
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
```

The same task concept is evaluated in both simulation and on the real robot.

## Simulation versions

The `simulation/code/` directory contains two generations of the task implementation.

### Initial version

- `lws_transfer_task.py`
- `lws_transfer.launch.py`
- `lws_transfer_params.yaml`

This is the initial implementation developed during the experiment. It is retained as the development version so that the repository preserves the original implementation and debugging history.

It provides the basic fixed-point transfer workflow and can still be launched independently.

### Final version

- `final_grasp_task.py`
- `final_grasp.launch.py`
- `final_grasp_params.yaml`

This is the final implementation used for the final simulation workflow.

Compared with the initial implementation, the final version reorganizes the task into clearer functional components, including robot kinematics, target solving, motion control, simulation-object handling, and high-level task execution.

The final version keeps the same experimental objective and uses the same robot model and simulation environment, while providing a more structured implementation for the final experiment.

## Why both versions are retained

Both versions are intentionally kept in the repository.

The initial version documents the development stage of the experiment, while the final version represents the finalized implementation. Keeping both versions makes the software evolution and experimental workflow easier to trace and reproduce.

The two task nodes are registered as separate ROS 2 executables, so one implementation does not overwrite the other.

## Simulation workflow

The simulation uses:

- ROS 2 Humble
- Gazebo / Ignition Fortress
- ros2_control
- ROS 2 trajectory action interfaces
- Robot state publishing
- Gazebo–ROS communication through `ros_gz_bridge`

The launch process starts the simulation world, robot model, state publisher, controllers, and task node in sequence.

The simulation parameters define:

- Robot base pose
- Point A and Point B
- Approach/safe height
- Tool-tip offset
- Gripper opening and closing positions
- Arm and gripper motion duration
- Number of repeated grasping cycles
- Simulation checking options

## Real-robot workflow

The `real_robot/` directory contains the corresponding implementation and records for physical-arm experiments.

The real-robot experiment includes:

- Robot driver startup
- Joint-state feedback
- Fixed-point grasping
- Execution logs
- Safety and abnormal-condition testing

The simulation is used to validate the task logic before transferring the workflow to the physical mechArm270.

## Repository structure

```text
Experiment_2/
├── simulation/
│   ├── code/
│   ├── config/
│   ├── logs/
│   ├── screenshots/
│   └── README.md
├── real_robot/
│   ├── code/
│   ├── config/
│   ├── logs/
│   ├── screenshots/
│   └── README.md
└── report/              # LaTeX report will be added in the next commit
```

## Workflow
Home → above Point A → descend → close gripper → lift → Point B → release → Home.

## Simulation evidence

### Gazebo scene
![Simulation scene](simulation/screenshots/01_simulation_scene.png)

### Task execution
![Simulation execution](simulation/screenshots/03_task_execution.png)

### Terminal / result evidence
![Simulation terminal](simulation/screenshots/04_terminal_execution.png)

## Real-robot evidence

### Real-robot task execution
![Real robot execution](real_robot/screenshots/01_real_robot_execution.jpg)

### ROS 2 real-arm driver
![Driver started](real_robot/screenshots/02_driver_started.png)

### Joint-state feedback
![Joint states](real_robot/screenshots/03_joint_states.png)

### Safety / abnormal test
![Joint limit abort](real_robot/screenshots/04_joint_limit_abort.png)

## Validation
The real-robot validation record reports 5 successful runs out of 5, with no collision recorded. The repository also preserves trajectory / execution logs and an abnormal joint-limit test as safety evidence.

## Platform
Ubuntu 22.04 · ROS 2 Humble · Gazebo / Ignition · MoveIt 2 / ros2_control · Jetson Orin · Elephant Robotics mechArm270

## Note
Demonstration videos are submitted separately according to the course requirements. The LaTeX experiment report will be added under `report/` in a subsequent commit.
