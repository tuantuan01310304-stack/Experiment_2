# Experiment 2 — mechArm270 Fixed-Point Grasping

## 1. Project Overview

This project implements a complete fixed-point pick-and-place task using a **mechArm270 robotic arm** based on **ROS 2 Humble**.

The experiment was completed in two stages:

- **Simulation:** Gazebo / Ignition + ROS 2 + MoveIt 2 / ros2_control
- **Real Robot:** Jetson Orin + mechArm270 + ROS 2 real-robot driver

The complete manipulation workflow is:

**Home → Move above Point A → Descend → Close gripper → Lift → Move to Point B → Release → Return Home**

The task uses predefined pick-and-place positions and does not rely on visual localization.

The project verifies not only successful fixed-point grasping, but also repeated execution, trajectory recording, ROS 2 joint-state feedback, and safety handling for abnormal or unreachable commands.

---

## 2. System Architecture

The experiment uses ROS 2 as the common control framework for both simulation and real-robot execution.

```text
                    ROS 2 Task Node
                           │
                           ▼
              FollowJointTrajectory
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼                           ▼
      Simulation Backend          Real-Robot Backend
      Gazebo / ros2_control       mecharm_real driver
             │                           │
             ▼                           ▼
      Simulated mechArm270        Jetson Orin
                                         │
                                         ▼
                                   mechArm270 Robot
```

The simulation stage is used to verify the motion sequence, target positions, trajectory execution, and exception handling before operating the physical robot.

For the real-robot stage, the ROS 2 trajectory-control architecture is retained while the hardware communication backend and workspace parameters are changed for the physical mechArm270.

---

## 3. Simulation

### 3.1 Simulation Environment

The simulation environment contains:

- mechArm270 robotic arm
- Adaptive gripper
- Work table
- Target object
- Fixed pick point A
- Fixed place point B
- Safe approach height

The environment was built using **Ubuntu 22.04**, **ROS 2 Humble**, **Gazebo / Ignition**, **MoveIt 2**, and **ros2_control**.

### Simulation Scene

![Gazebo Simulation Environment](screenshots/simulation_scene.png)

*Figure 1. Gazebo simulation environment for the mechArm270 fixed-point grasping task.*

### Gazebo Model and Scene Setup

![Gazebo Scene Setup](screenshots/gazebo_scene_setup.png)

*Figure 2. Gazebo scene containing the mechArm270, table, target object, and simulation environment.*

---

## 4. Fixed-Point Pick-and-Place Configuration

The grasping task uses predefined target positions rather than visual localization.

The main simulation parameters include:

- **Point A:** object pick position
- **Point B:** object placement position
- **Safe height:** intermediate height used to reduce collision risk
- **Arm movement time:** trajectory execution duration
- **Gripper movement time:** gripper opening / closing duration
- **Repeat count:** 5

### Task Parameters

![Fixed Point Parameters](screenshots/fixed_point_parameters.png)

*Figure 3. Fixed-point grasping parameters including Point A, Point B, safe height, motion time, and repeat count.*

The basic motion sequence is:

```text
HOME
  ↓
Move above Point A
  ↓
Descend to Point A
  ↓
Close gripper
  ↓
Lift to safe height
  ↓
Move toward Point B
  ↓
Descend to Point B
  ↓
Open gripper
  ↓
Return HOME
```

Using an intermediate safe height prevents the end effector and the grasped object from moving directly through the work surface during horizontal transfer.

---

## 5. Simulation Validation

The simulation task was executed repeatedly to verify the stability of the fixed-point grasping workflow.

Five consecutive grasping cycles were recorded.

### Five-Run Simulation Test

![Five Run Simulation Result](screenshots/simulation_five_runs.png)

*Figure 4. Five-run simulation validation result.*

The recorded summary shows:

```text
Cycle 1: Success
Cycle 2: Success
Cycle 3: Success
Cycle 4: Success
Cycle 5: Success

Completed: 5 / 5
```

Therefore, the simulation achieved a **100% completion rate in the recorded five-run validation**.

Trajectory data were also recorded during execution for subsequent analysis and verification.

---

## 6. Simulation Exception Handling

In addition to normal grasping execution, abnormal conditions were tested to verify that unsafe or invalid commands would not cause uncontrolled motion.

The tested abnormal conditions included:

- Communication / trajectory target timeout
- Invalid or unavailable joint solution
- Unreachable target position
- Trajectory execution timeout

When an invalid target was detected, the task was rejected or stopped and an error message was recorded.

### Exception Test Record

![Simulation Exception Test](screenshots/simulation_exception_test.png)

*Figure 5. Simulation exception and safety-handling records.*

This mechanism prevents the robot from blindly executing an invalid trajectory when a valid solution cannot be obtained.

---

## 7. Real-Robot Implementation

After simulation validation, the task was transferred to the physical **mechArm270** platform.

The real-robot system consists of:

- Jetson Orin
- ROS 2 Humble
- Elephant Robotics mechArm270
- Adaptive gripper
- ROS 2 real-robot driver
- Fixed A/B workspace positions

The real-robot control chain is:

```text
ROS 2 Task Node
       │
       ▼
FollowJointTrajectory
       │
       ▼
mecharm_real Driver
       │
       ▼
Jetson Orin
       │
       ▼
mechArm270
```

The real-robot stage preserves the ROS 2 trajectory-control architecture used in the simulation while adapting the communication and position parameters to the physical robot.

---

## 8. ROS 2 Real-Robot Driver

The real-robot driver connects ROS 2 trajectory commands to the physical mechArm270.

### Driver Startup

![Real Robot Driver](screenshots/real_robot_driver.png)

*Figure 6. ROS 2 mechArm270 real-robot driver successfully started on the Jetson platform.*

The driver provides the hardware interface required for executing the same high-level fixed-point manipulation workflow on the real robotic arm.

---

## 9. Joint-State Feedback

The physical robot publishes its current joint positions through ROS 2.

The `/joint_states` topic contains the six arm joints and the gripper controller.

### Joint-State Feedback

![Joint States](screenshots/joint_states.png)

*Figure 7. ROS 2 `/joint_states` feedback from the physical mechArm270.*

This feedback provides confirmation that the software can obtain the current state of the real robot instead of operating only through open-loop commands.

---

## 10. Real-Robot Grasping Results

The real-robot fixed-point grasping task was tested repeatedly.

### Five Independent A → B Tests

| Run | Result | Execution Time |
|---:|:---:|---:|
| 1 | Success | 36.026 s |
| 2 | Success | 36.038 s |
| 3 | Success | 36.060 s |
| 4 | Success | 35.914 s |
| 5 | Success | 35.954 s |
| **Total** | **5 / 5 Success** | **Average: 35.998 s** |

The five independent real-robot tests achieved a **100% success rate**.

In addition, a continuous bidirectional transfer test was performed:

```text
A → B → A → B → A → B
```

All five transfer segments were completed successfully.

**Total execution time: 194.076 s**

### Continuous Real-Robot Transfer

![Real Robot Continuous Transfer](screenshots/real_robot_transfer.png)

*Figure 8. ROS 2 real-robot continuous transfer test.*

---

## 11. Safety Validation

Safety handling was also verified on the physical robot.

An intentionally invalid target was sent to `joint1_to_base`:

```text
Target position: 2.967 rad
```

The target exceeded the configured URDF joint limit.

The trajectory action rejected the command and returned:

```text
error_code: -5
Goal finished with status: ABORTED
```

The robot joint position remained unchanged before and after the rejected command, demonstrating that the invalid target did not result in physical robot motion.

### Joint-Limit Safety Test

![Joint Limit Safety Test](screenshots/joint_limit_abort.png)

*Figure 9. Out-of-limit joint command rejected by the ROS 2 real-robot control system.*

This test verifies that the system can identify an unsafe trajectory target and stop the corresponding task instead of executing an invalid motion.

---

## 12. Experimental Results Summary

| Test | Result |
|---|---|
| Gazebo fixed-point grasping workflow | Completed |
| Simulation repeated grasping | 5 / 5 completed |
| Simulation trajectory recording | Completed |
| Simulation exception handling | Completed |
| ROS 2 real-robot driver | Completed |
| Real-robot joint-state feedback | Completed |
| Real-robot A → B grasping | 5 / 5 successful |
| Average real-robot execution time | 35.998 s |
| Continuous real-robot transfer | 5 segments completed |
| Continuous transfer time | 194.076 s |
| Joint-limit safety test | ABORTED correctly |
| Demonstration videos | Completed |

---

## 13. Repository Structure

```text
Experiment_2/
│
├── simulation/
│   ├── code/
│   ├── logs/
│   ├── screenshots/
│   └── report_materials/
│
├── real_robot/
│   ├── code_driver/
│   ├── code_grasp/
│   └── logs/
│
├── report/
│   ├── figures/
│   ├── experiment_report.tex
│   └── experiment_report.pdf
│
├── screenshots/
│
├── .gitignore
└── README.md
```

### Directory Description

- `simulation/` — Gazebo simulation packages, configuration files, logs, screenshots, and validation materials.
- `real_robot/` — ROS 2 real-robot driver, grasping task, parameters, and experimental logs.
- `report/` — LaTeX source, report figures, and final PDF report.
- `screenshots/` — Images displayed in this README.
- `README.md` — Project documentation and experimental summary.

---

## 14. Main Software and Hardware

### Software

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo / Ignition
- MoveIt 2
- ros2_control
- Python 3

### Hardware

- NVIDIA Jetson Orin
- Elephant Robotics mechArm270
- Adaptive gripper
- Fixed work surface and target object

---

## 15. Key Features

- Complete simulation-to-real-robot workflow
- ROS 2 based trajectory control
- Fixed-point pick-and-place task
- Gazebo simulation validation before real-robot operation
- Repeated grasping validation
- Joint-state feedback
- Trajectory and result logging
- Unreachable-target handling
- Joint-limit safety protection
- Real-robot repeated execution
- LaTeX-based experiment documentation

---

## 16. Experiment Report

The final experiment report is written in **LaTeX** as required by the course.

The report includes:

- Experimental objectives
- System architecture
- Simulation design
- Fixed-point grasping workflow
- Real-robot implementation
- Experimental results
- Five-run validation
- Exception and safety tests
- GitHub repository and commit records
- Analysis and conclusion

The LaTeX source and compiled PDF will be available in:

```text
report/
```

---

## 17. Demonstration

Simulation and real-robot demonstration videos were recorded separately as part of the experiment submission materials.

The videos demonstrate:

- Gazebo fixed-point grasping
- Repeated simulation execution
- Real mechArm270 fixed-point grasping
- Continuous real-robot transfer

---

## 18. Authors

Group members:

```text
Name: ____________________
Student ID: ______________

Name: ____________________
Student ID: ______________
```

---

## 19. Course Submission

This repository contains the code, configuration, experimental records, screenshots, and LaTeX report materials for **Experiment 2**.

Git commit history is retained to document the development and submission process.
