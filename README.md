# Experiment 2 — mechArm270 Fixed-Point Grasping

ROS 2 Humble based fixed-point pick-and-place experiment using Gazebo / Ignition simulation and a real mechArm270 robotic arm.

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
