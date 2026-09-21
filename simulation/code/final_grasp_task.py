#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MechArm 270 定点抓取与搬运任务节点

程序结构：
1. RobotModel：负责机械臂正运动学、TCP位置和数值雅可比。
2. TargetSolver：根据当前关节状态迭代求取目标姿态。
3. MotionInterface：负责 ROS2 FollowJointTrajectory 通信。
4. SimulationObject：负责 Gazebo 中目标物体的读取、复位和跟随。
5. TransferController：负责整个抓取任务的状态管理和日志记录。

任务流程：
准备 → 接近取物点 → 抓取 → 抬升 → 搬运 → 接近放置点
→ 释放 → 离开 → 回零。

说明：
运动学中的几何参数来自当前 MechArm 270 模型，因此这些数值不能
为了降低代码相似性而随意修改，否则会直接改变机械臂运动结果。
"""

import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ============================================================
# 机器人基本配置
# ============================================================

MANIPULATOR_JOINT_NAMES = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
]

END_EFFECTOR_JOINT = "gripper_controller"

MANIPULATOR_LIMITS = np.array([
    [-2.792527, 2.792527],
    [-1.3089,   2.0943],
    [-3.0543,   1.1344],
    [-2.7052,   2.7052],
    [-2.0071,   2.0071],
    [-3.14,     3.14],
], dtype=float)


# ============================================================
# 运动学模型
# ============================================================

def rotation_from_rpy(roll, pitch, yaw):
    """将 URDF 使用的 RPY 参数转换为旋转矩阵。"""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def homogeneous_translation_rotation(position, rpy):
    """根据平移和旋转建立齐次变换矩阵。"""
    transform = np.eye(4)
    transform[:3, :3] = rotation_from_rpy(*rpy)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def z_axis_rotation(angle):
    """建立绕局部 Z 轴旋转的齐次矩阵。"""
    c = math.cos(angle)
    s = math.sin(angle)

    transform = np.eye(4)
    transform[0, 0] = c
    transform[0, 1] = -s
    transform[1, 0] = s
    transform[1, 1] = c
    return transform


# 机械臂各关节 origin 参数。
# 这些参数对应当前仿真模型，不应仅为了改代码外观而改变。
JOINT_ORIGINS = [
    homogeneous_translation_rotation([0.0, 0.0, 0.1], [0.0, 0.0, 0.0]),
    homogeneous_translation_rotation([0.0, 0.0, 0.038], [-1.5708, 0.0, 0.0]),
    homogeneous_translation_rotation([0.0, -0.1, 0.0], [0.0, 0.0, 0.0]),
    homogeneous_translation_rotation([0.108, -0.005, -0.001], [0.0, 1.5708, 0.0]),
    homogeneous_translation_rotation([-0.001, 0.0, 0.0], [0.0, -1.5708, 0.0]),
    homogeneous_translation_rotation([0.06, 0.0, 0.0], [0.0, 1.5708, 0.0]),
]

GRIPPER_BASE_OFFSET = homogeneous_translation_rotation(
    [0.0, 0.0, 0.038],
    [1.579, 0.0, 0.0],
)


class RobotModel:
    """机械臂几何模型。"""

    def __init__(self, tcp_offset):
        self.tcp_offset = float(tcp_offset)

    def forward(self, joint_vector):
        """计算腕部和夹爪基座位姿。"""
        q = np.asarray(joint_vector, dtype=float)

        transform = np.eye(4)

        for index in range(6):
            transform = (
                transform
                @ JOINT_ORIGINS[index]
                @ z_axis_rotation(q[index])
            )

        wrist_transform = transform
        end_effector_transform = transform @ GRIPPER_BASE_OFFSET

        return wrist_transform, end_effector_transform

    def tcp_state(self, joint_vector):
        """返回 TCP 位置和工具接近方向。"""
        wrist, gripper = self.forward(joint_vector)

        tool_axis = wrist[:3, 2]
        tcp_position = (
            gripper[:3, 3]
            + tool_axis * self.tcp_offset
        )

        return tcp_position, tool_axis

    def numerical_jacobian(self, joint_vector, step=1e-5):
        """通过有限差分获得 TCP 位置与工具方向的雅可比矩阵。"""
        q = np.asarray(joint_vector, dtype=float)
        base_position, base_axis = self.tcp_state(q)

        jacobian = np.zeros((6, 6), dtype=float)

        for joint_index in range(6):
            shifted = q.copy()
            shifted[joint_index] += step

            new_position, new_axis = self.tcp_state(shifted)

            jacobian[:, joint_index] = np.concatenate([
                (new_position - base_position) / step,
                (new_axis - base_axis) / step,
            ])

        return jacobian


class TargetSolver:
    """基于当前机械臂姿态进行连续迭代的目标求解器。"""

    def __init__(self, robot_model):
        self.robot = robot_model

    @staticmethod
    def _within_limits(q):
        return bool(
            np.all(q >= MANIPULATOR_LIMITS[:, 0])
            and np.all(q <= MANIPULATOR_LIMITS[:, 1])
        )

    @staticmethod
    def _initial_configuration(target, current):
        """根据当前姿态生成初始关节向量。"""
        q = np.asarray(current, dtype=float).copy()

        if q.shape != (6,):
            q = np.zeros(6, dtype=float)

        horizontal_angle = math.atan2(target[1], target[0])

        # 第一关节主要决定水平面方向。
        if abs(target[0]) + abs(target[1]) > 1e-5:
            q[0] = horizontal_angle

        return np.clip(
            q,
            MANIPULATOR_LIMITS[:, 0],
            MANIPULATOR_LIMITS[:, 1],
        )

    def solve(self, target, current, max_iterations=300):
        """
        从当前关节状态开始迭代。

        目标：
        1. TCP 接近目标位置；
        2. 工具轴尽量朝向竖直向下；
        3. 每次关节变化受到限制；
        4. 最终结果必须满足机械关节范围。
        """
        target = np.asarray(target, dtype=float)

        q = self._initial_configuration(target, current)

        desired_axis = np.array([0.0, 0.0, -1.0])
        position_tolerance = 0.002
        direction_tolerance = 0.26

        position_weight = 1.0
        direction_weight = 0.30

        damping = 1e-3
        max_joint_step = 0.20

        best_q = q.copy()
        best_error = float("inf")

        for _ in range(max_iterations):
            current_position, current_axis = self.robot.tcp_state(q)

            position_error = target - current_position
            direction_error = desired_axis - current_axis

            combined_error = np.concatenate([
                position_weight * position_error,
                direction_weight * direction_error,
            ])

            position_norm = np.linalg.norm(position_error)
            direction_norm = np.linalg.norm(direction_error)

            total_error = (
                position_norm
                + direction_weight * direction_norm
            )

            if total_error < best_error:
                best_error = total_error
                best_q = q.copy()

            if (
                position_norm <= position_tolerance
                and direction_norm <= direction_tolerance
            ):
                return q

            jacobian = self.robot.numerical_jacobian(q)

            normal_matrix = (
                jacobian.T @ jacobian
                + damping * np.eye(6)
            )

            try:
                delta_q = np.linalg.solve(
                    normal_matrix,
                    jacobian.T @ combined_error,
                )
            except np.linalg.LinAlgError:
                return None

            delta_q = np.clip(
                delta_q,
                -max_joint_step,
                max_joint_step,
            )

            candidate = q + delta_q
            candidate = np.clip(
                candidate,
                MANIPULATOR_LIMITS[:, 0],
                MANIPULATOR_LIMITS[:, 1],
            )

            if not self._within_limits(candidate):
                return None

            q = candidate

        final_position, final_axis = self.robot.tcp_state(best_q)

        if (
            np.linalg.norm(target - final_position) <= 0.004
            and np.linalg.norm(desired_axis - final_axis) <= 0.35
        ):
            return best_q

        return None


# ============================================================
# 任务规划数据
# ============================================================

class TaskPlanner:
    """根据取物点和放置点生成任务所需的目标姿态。"""

    def __init__(
        self,
        robot_model,
        solver,
        base_position,
        pickup_position,
        dropoff_position,
        approach_height,
        tcp_offset,
    ):
        self.robot = robot_model
        self.solver = solver

        self.base_position = np.asarray(base_position, dtype=float)
        self.pickup_position = np.asarray(pickup_position, dtype=float)
        self.dropoff_position = np.asarray(dropoff_position, dtype=float)

        self.approach_height = float(approach_height)
        self.tcp_offset = float(tcp_offset)

    def _relative(self, world_position):
        return np.asarray(world_position, dtype=float) - self.base_position

    def _make_target(self, position, extra_z):
        local = self._relative(position)
        local = local.copy()
        local[2] += extra_z
        return local

    def _check_reachability(self, target):
        reference = np.array([0.0, 0.0, 0.138])
        return np.linalg.norm(target - reference) <= 0.30

    def _solve_sequence(self, targets):
        result = {}
        current_guess = np.zeros(6, dtype=float)

        for name, target in targets:
            if not self._check_reachability(target):
                return None, f"{name} 超出工作范围"

            solution = self.solver.solve(
                target,
                current_guess,
            )

            if solution is None:
                return None, f"{name} 无法获得有效关节解"

            result[name] = solution
            current_guess = solution.copy()

        result["home"] = np.zeros(6, dtype=float)

        return result, ""

    def generate(self):
        pickup_above = self._make_target(
            self.pickup_position,
            self.approach_height,
        )
        pickup_contact = self._make_target(
            self.pickup_position,
            0.002,
        )

        dropoff_above = self._make_target(
            self.dropoff_position,
            self.approach_height,
        )
        dropoff_contact = self._make_target(
            self.dropoff_position,
            0.008,
        )

        targets = [
            ("pickup_approach", pickup_above),
            ("pickup_contact", pickup_contact),
            ("dropoff_approach", dropoff_above),
            ("dropoff_contact", dropoff_contact),
        ]

        return self._solve_sequence(targets)


# ============================================================
# ROS2 动作通信
# ============================================================

class MotionInterface:
    """封装机械臂和夹爪 FollowJointTrajectory 控制。"""

    def __init__(self, node, callback_group):
        self.node = node

        self.arm_action = ActionClient(
            node,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            callback_group=callback_group,
        )

        self.gripper_action = ActionClient(
            node,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            callback_group=callback_group,
        )

    def wait_until_ready(self):
        arm_ready = self.arm_action.wait_for_server(timeout_sec=60)
        gripper_ready = self.gripper_action.wait_for_server(timeout_sec=60)
        return arm_ready and gripper_ready

    def _validate_arm_target(self, joint_names, values):
        for name, value in zip(joint_names, values):
            if name not in MANIPULATOR_JOINT_NAMES:
                continue

            index = MANIPULATOR_JOINT_NAMES.index(name)
            lower, upper = MANIPULATOR_LIMITS[index]

            if value < lower - 1e-6 or value > upper + 1e-6:
                return False, (
                    f"{name} 目标 {value:.4f} 超出范围 "
                    f"[{lower:.4f}, {upper:.4f}]"
                )

        return True, ""

    def send(self, action_client, joint_names, positions, duration):
        valid, reason = self._validate_arm_target(
            joint_names,
            positions,
        )

        if not valid:
            self.node.report_error(reason)
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]

        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)

        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds

        goal.trajectory.points = [point]

        future = action_client.send_goal_async(goal)

        start = time.time()

        while not future.done():
            time.sleep(0.05)

            if time.time() - start > 10:
                self.node.report_error("等待轨迹目标响应超时")
                return False

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.node.report_error("控制器拒绝了轨迹目标")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()

        while not result_future.done():
            time.sleep(0.05)

            if time.time() - start > duration * 10 + 60:
                self.node.report_error("轨迹执行超时")
                return False

        result = result_future.result().result

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.report_error(
                f"轨迹执行失败：{result.error_code} "
                f"{result.error_string}"
            )
            return False

        return True

    def move_arm(self, joint_vector, duration):
        return self.send(
            self.arm_action,
            MANIPULATOR_JOINT_NAMES,
            joint_vector,
            duration,
        )

    def move_gripper(self, position, duration):
        return self.send(
            self.gripper_action,
            [END_EFFECTOR_JOINT],
            [position],
            duration,
        )


# ============================================================
# Gazebo 目标物体管理
# ============================================================

class SimulationObject:
    """处理 Gazebo 中目标方块的位置和跟随控制。"""

    def __init__(self, node, world_name, robot_model):
        self.node = node
        self.world_name = world_name
        self.robot = robot_model

        self.position = None
        self.follow_enabled = False
        self.place_target = None

        self.velocity_publisher = node.create_publisher(
            Twist,
            "/model/target_object/cmd_vel",
            10,
        )

    def update_odometry(self, message):
        point = message.pose.pose.position

        self.position = np.array([
            point.x,
            point.y,
            point.z,
        ], dtype=float)

    def reset(self, world_position):
        request = (
            f'name: "target_object", '
            f'position: {{'
            f'x: {world_position[0]:.4f}, '
            f'y: {world_position[1]:.4f}, '
            f'z: {world_position[2]:.4f}'
            f'}}'
        )

        try:
            subprocess.run(
                [
                    "ign",
                    "service",
                    "-s",
                    f"/world/{self.world_name}/set_pose",
                    "--reqtype",
                    "ignition.msgs.Pose",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "200",
                    "--req",
                    request,
                ],
                capture_output=True,
                timeout=2,
            )
            return True
        except Exception as exc:
            self.node.report_error(
                f"目标物体复位失败：{exc}"
            )
            return False

    def read_world_pose(self):
        try:
            result = subprocess.run(
                [
                    "ign",
                    "topic",
                    "-e",
                    "-n",
                    "1",
                    "-t",
                    f"/world/{self.world_name}/pose/info",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )

            text = result.stdout

            for block in text.split("pose {"):
                if '"target_object"' not in block:
                    continue

                match = re.search(
                    r"position\s*{\s*"
                    r"x:\s*([-\d.e]+)\s*"
                    r"y:\s*([-\d.e]+)\s*"
                    r"z:\s*([-\d.e]+)",
                    block,
                )

                if match:
                    return np.array([
                        float(match.group(1)),
                        float(match.group(2)),
                        float(match.group(3)),
                    ])

        except Exception as exc:
            self.node.report_error(
                f"读取目标物体位置失败：{exc}"
            )

        return None

    def control_tick(self, joint_state_map, base_position):
        if not self.follow_enabled:
            return

        if self.position is None:
            return

        if self.place_target is not None:
            desired = np.asarray(
                self.place_target,
                dtype=float,
            )
        else:
            current = [
                joint_state_map.get(name)
                for name in MANIPULATOR_JOINT_NAMES
            ]

            if any(value is None for value in current):
                return

            tcp, _ = self.robot.tcp_state(
                np.asarray(current, dtype=float)
            )

            desired = base_position + tcp

        velocity = np.clip(
            6.0 * (desired - self.position),
            -0.6,
            0.6,
        )

        command = Twist()
        command.linear.x = float(velocity[0])
        command.linear.y = float(velocity[1])
        command.linear.z = float(velocity[2])

        self.velocity_publisher.publish(command)

    def stop(self):
        self.follow_enabled = False
        self.place_target = None

        for _ in range(3):
            self.velocity_publisher.publish(Twist())
            time.sleep(0.05)


# ============================================================
# 任务执行器
# ============================================================

class TransferController(Node):
    """整个抓取任务的执行控制器。"""

    def __init__(self):
        super().__init__("transfer_controller")

        self._declare_parameters()
        self._load_parameters()

        self.callback_group = ReentrantCallbackGroup()

        self.robot = RobotModel(self.tcp_offset)
        self.solver = TargetSolver(self.robot)

        self.motion = MotionInterface(
            self,
            self.callback_group,
        )

        self.sim_object = SimulationObject(
            self,
            self.world_name,
            self.robot,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/grasp_status",
            10,
        )

        self.joint_state_map = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Odometry,
            "/model/target_object/odometry",
            self.sim_object.update_odometry,
            10,
            callback_group=self.callback_group,
        )

        self._open_logs()

        self.control_timer = self.create_timer(
            1.0 / 30.0,
            self._simulation_tick,
            callback_group=self.callback_group,
        )

        self.worker = threading.Thread(
            target=self.execute_task,
            daemon=True,
        )
        self.worker.start()

    def _declare_parameters(self):
        parameters = [
            ("base_pose_xyz", [0.0, 0.0, 0.75]),
            ("pickup_position", [0.16, 0.09, 0.7635]),
            ("dropoff_position", [0.16, -0.09, 0.7635]),
            ("approach_height", 0.08),
            ("tool_tip_offset", 0.10),
            ("gripper_release_position", 0.1),
            ("gripper_hold_position", -0.55),
            ("arm_motion_time", 2.5),
            ("gripper_motion_time", 1.2),
            ("task_repeat_limit", 5),
            ("log_dir", "/ws/grasp_logs"),
            ("world_name", "grasp_world"),
            ("sim_attach", True),
            ("sim_check", True),
            ("alternate_direction", False),
            ("use_taught_joints", False),
            ("pickup_joints_deg", [0.0] * 6),
            ("dropoff_joints_deg", [0.0] * 6),
            ("lift_angle_deg", 25.0),
            ("drop_lift_angle_deg", 1.5),
        ]

        for name, default in parameters:
            self.declare_parameter(name, default)

    def _load_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self.base_position = np.asarray(
            value("base_pose_xyz"),
            dtype=float,
        )

        self.pickup_position = np.asarray(
            value("pickup_position"),
            dtype=float,
        )

        self.dropoff_position = np.asarray(
            value("dropoff_position"),
            dtype=float,
        )

        self.approach_height = float(
            value("approach_height")
        )

        self.tcp_offset = float(
            value("tool_tip_offset")
        )

        self.gripper_release_position = float(
            value("gripper_release_position")
        )

        self.gripper_hold_position = float(
            value("gripper_hold_position")
        )

        self.arm_motion_time = float(
            value("arm_motion_time")
        )

        self.gripper_motion_time = float(
            value("gripper_motion_time")
        )

        self.task_repeat_limit = int(
            value("task_repeat_limit")
        )

        self.world_name = str(
            value("world_name")
        )

        self.sim_attach = bool(
            value("sim_attach")
        )

        self.sim_check = bool(
            value("sim_check")
        )

        self.alternate_direction = bool(
            value("alternate_direction")
        )

        self.use_taught_joints = bool(
            value("use_taught_joints")
        )

        self.pickup_joints_deg = list(
            value("pickup_joints_deg")
        )

        self.dropoff_joints_deg = list(
            value("dropoff_joints_deg")
        )

        self.lift_angle_deg = float(
            value("lift_angle_deg")
        )

        self.drop_lift_angle_deg = float(
            value("drop_lift_angle_deg")
        )

        self.log_dir = os.path.expanduser(
            str(value("log_dir"))
        )

        os.makedirs(
            self.log_dir,
            exist_ok=True,
        )

    def _open_logs(self):
        self.trajectory_log = open(
            os.path.join(
                self.log_dir,
                "trajectory.csv",
            ),
            "w",
        )

        self.trajectory_log.write(
            "stamp,"
            + ",".join(
                MANIPULATOR_JOINT_NAMES
                + [END_EFFECTOR_JOINT]
            )
            + "\n"
        )

        self.result_log = open(
            os.path.join(
                self.log_dir,
                "results.csv",
            ),
            "w",
        )

        self.result_log.write(
            "cycle,success,x,y,z,detail\n"
        )

        self.error_log = open(
            os.path.join(
                self.log_dir,
                "errors.log",
            ),
            "a",
        )

    def publish_status(self, message):
        self.get_logger().info(message)
        self.status_publisher.publish(
            String(data=message)
        )

    def report_error(self, message):
        self.get_logger().error(message)

        if hasattr(self, "error_log"):
            self.error_log.write(
                f"{datetime.now().isoformat()} {message}\n"
            )
            self.error_log.flush()

        self.status_publisher.publish(
            String(data="ERROR: " + message)
        )

    def _joint_state_callback(self, message):
        for name, position in zip(
            message.name,
            message.position,
        ):
            self.joint_state_map[name] = position

        if (
            hasattr(self, "trajectory_log")
            and not self.trajectory_log.closed
        ):
            values = []

            for name in (
                MANIPULATOR_JOINT_NAMES
                + [END_EFFECTOR_JOINT]
            ):
                values.append(
                    f"{self.joint_state_map.get(name, float('nan')):.4f}"
                )

            timestamp = (
                message.header.stamp.sec
                + message.header.stamp.nanosec * 1e-9
            )

            self.trajectory_log.write(
                f"{timestamp:.3f},"
                + ",".join(values)
                + "\n"
            )

    def _simulation_tick(self):
        self.sim_object.control_tick(
            self.joint_state_map,
            self.base_position,
        )

    def _current_arm_configuration(self):
        values = [
            self.joint_state_map.get(name)
            for name in MANIPULATOR_JOINT_NAMES
        ]

        if any(value is None for value in values):
            return None

        return np.asarray(values, dtype=float)

    def _build_taught_plan(self):
        plan = {}

        for label, degrees in (
            ("pickup", self.pickup_joints_deg),
            ("dropoff", self.dropoff_joints_deg),
        ):
            if len(degrees) != 6:
                self.report_error(
                    f"{label} 示教数据必须包含 6 个关节角"
                )
                return None

            contact = np.radians(
                np.asarray(degrees, dtype=float)
            )

            approach = contact.copy()
            approach[1] -= math.radians(
                self.lift_angle_deg
            )

            release = contact.copy()
            release[1] -= math.radians(
                self.drop_lift_angle_deg
            )

            for name, configuration in (
                (f"{label}_contact", contact),
                (f"{label}_approach", approach),
                (f"{label}_release", release),
            ):
                if not np.all(
                    (configuration >= MANIPULATOR_LIMITS[:, 0])
                    & (configuration <= MANIPULATOR_LIMITS[:, 1])
                ):
                    self.report_error(
                        f"{name} 存在超出机械关节范围的角度"
                    )
                    return None

                plan[name] = configuration

        plan["home"] = np.zeros(6)

        return plan

    def create_motion_plan(self):
        if self.use_taught_joints:
            return self._build_taught_plan()

        planner = TaskPlanner(
            self.robot,
            self.solver,
            self.base_position,
            self.pickup_position,
            self.dropoff_position,
            self.approach_height,
            self.tcp_offset,
        )

        plan, reason = planner.generate()

        if plan is None:
            self.report_error(
                f"路径规划失败：{reason}"
            )

        return plan

    def _wait_for_joint_states(self):
        start = time.time()

        while True:
            if all(
                name in self.joint_state_map
                for name in MANIPULATOR_JOINT_NAMES
            ):
                return True

            if time.time() - start > 30:
                return False

            time.sleep(0.2)

    def _move_home(self, plan):
        return self.motion.move_arm(
            plan["home"],
            self.arm_motion_time,
        )

    def _move_to_pickup(self, plan, source):
        if source == "pickup":
            return self.motion.move_arm(
                plan["pickup_approach"],
                self.arm_motion_time,
            )

        return self.motion.move_arm(
            plan["dropoff_approach"],
            self.arm_motion_time,
        )

    def _move_to_contact(self, plan, source):
        if source == "pickup":
            target = plan["pickup_contact"]
        else:
            target = plan["dropoff_contact"]

        return self.motion.move_arm(
            target,
            self.arm_motion_time * 0.6,
        )

    def _lift_from_source(self, plan, source):
        if source == "pickup":
            target = plan["pickup_approach"]
        else:
            target = plan["dropoff_approach"]

        return self.motion.move_arm(
            target,
            self.arm_motion_time * 0.6,
        )

    def _move_to_dropoff(self, plan, destination):
        if destination == "dropoff":
            target = plan["dropoff_approach"]
        else:
            target = plan["pickup_approach"]

        return self.motion.move_arm(
            target,
            self.arm_motion_time,
        )

    def _move_to_drop_contact(self, plan, destination):
        if destination == "dropoff":
            target = plan["dropoff_contact"]
        else:
            target = plan["pickup_contact"]

        return self.motion.move_arm(
            target,
            self.arm_motion_time * 0.6,
        )

    def _leave_destination(self, plan, destination):
        if destination == "dropoff":
            target = plan["dropoff_approach"]
        else:
            target = plan["pickup_approach"]

        return self.motion.move_arm(
            target,
            self.arm_motion_time * 0.6,
        )

    def _grasp(self):
        if self.sim_attach:
            self.sim_object.follow_enabled = True
            self.sim_object.place_target = None
            time.sleep(0.4)

        success = self.motion.move_gripper(
            self.gripper_hold_position,
            self.gripper_motion_time,
        )

        if not success:
            self.sim_object.stop()
            return False

        if not self.sim_attach:
            self.sim_object.follow_enabled = True

        return True

    def _release(self, destination):
        if self.sim_attach:
            if destination == "dropoff":
                self.sim_object.place_target = self.dropoff_position
            else:
                self.sim_object.place_target = self.pickup_position

            time.sleep(0.8)
            self.sim_object.stop()

        else:
            self.sim_object.follow_enabled = False

        return self.motion.move_gripper(
            self.gripper_release_position,
            self.gripper_motion_time,
        )

    def _execute_stage(self, cycle, description, callback):
        self.publish_status(
            f"[第 {cycle} 次] {description}"
        )

        try:
            success = callback()
        except Exception as exc:
            self.report_error(
                f"{description} 执行异常：{exc}"
            )
            success = False

        if not success:
            self.result_log.write(
                f"{cycle},0,,,,失败：{description}\n"
            )
            self.result_log.flush()

        return success

    def _verify_placement(self, cycle, destination):
        if not self.sim_check:
            self.result_log.write(
                f"{cycle},1,,,,动作完成，真机需人工确认落点\n"
            )
            self.result_log.flush()
            return True

        target = (
            self.dropoff_position
            if destination == "dropoff"
            else self.pickup_position
        )

        actual = self.sim_object.read_world_pose()

        if actual is None:
            self.result_log.write(
                f"{cycle},0,,,,无法读取物体最终位置\n"
            )
            self.result_log.flush()
            return False

        distance = np.linalg.norm(
            actual[:2] - target[:2]
        )

        success = distance < 0.03

        detail = (
            "放置位置满足要求"
            if success
            else "物体没有进入目标区域"
        )

        self.result_log.write(
            f"{cycle},{int(success)},"
            f"{actual[0]:.4f},{actual[1]:.4f},{actual[2]:.4f},"
            f"{detail}\n"
        )
        self.result_log.flush()

        return success

    def _execute_cycle(self, cycle, plan, source, destination):
        source_label = (
            "取物点"
            if source == "pickup"
            else "放置点"
        )

        destination_label = (
            "放置点"
            if destination == "dropoff"
            else "取物点"
        )

        stages = [
            (
                "回到初始姿态",
                lambda: self._move_home(plan),
            ),
            (
                "打开夹爪",
                lambda: self.motion.move_gripper(
                    self.gripper_release_position,
                    self.gripper_motion_time,
                ),
            ),
            (
                f"移动到{source_label}上方",
                lambda: self._move_to_pickup(
                    plan,
                    source,
                ),
            ),
            (
                f"下降到{source_label}",
                lambda: self._move_to_contact(
                    plan,
                    source,
                ),
            ),
            (
                "执行抓取",
                self._grasp,
            ),
            (
                "抬升目标物体",
                lambda: self._lift_from_source(
                    plan,
                    source,
                ),
            ),
            (
                f"移动到{destination_label}上方",
                lambda: self._move_to_dropoff(
                    plan,
                    destination,
                ),
            ),
            (
                f"下降到{destination_label}",
                lambda: self._move_to_drop_contact(
                    plan,
                    destination,
                ),
            ),
            (
                "释放目标物体",
                lambda: self._release(
                    destination
                ),
            ),
            (
                "离开放置区域",
                lambda: self._leave_destination(
                    plan,
                    destination,
                ),
            ),
            (
                "返回初始姿态",
                lambda: self._move_home(plan),
            ),
        ]

        for description, callback in stages:
            if not self._execute_stage(
                cycle,
                description,
                callback,
            ):
                self.sim_object.stop()
                return False

        return self._verify_placement(
            cycle,
            destination,
        )

    def _reset_for_next_cycle(self, next_source):
        if not self.sim_check:
            return True

        target = (
            self.pickup_position
            if next_source == "pickup"
            else self.dropoff_position
        )

        for _ in range(4):
            if self.sim_object.reset(target):
                time.sleep(0.5)

                if (
                    self.sim_object.position is not None
                    and np.linalg.norm(
                        self.sim_object.position[:2]
                        - target[:2]
                    ) < 0.01
                ):
                    return True

        self.report_error(
            "下一轮任务开始前，目标物体无法复位"
        )
        return False

    def execute_task(self):
        try:
            time.sleep(2.0)

            self.publish_status(
                "等待机械臂和夹爪控制器上线..."
            )

            if not self.motion.wait_until_ready():
                self.report_error(
                    "FollowJointTrajectory 控制器未上线"
                )
                return

            self.publish_status(
                "等待关节状态..."
            )

            if not self._wait_for_joint_states():
                self.report_error(
                    "30 秒内没有获得完整机械臂关节状态"
                )
                return

            if self.sim_check:
                object_pose = (
                    self.sim_object.read_world_pose()
                )

                if object_pose is None:
                    self.report_error(
                        "没有检测到 Gazebo 中的 target_object"
                    )
                    return

                self.publish_status(
                    "目标物体当前位置："
                    + str(np.round(object_pose, 4).tolist())
                )

            self.publish_status(
                "开始生成机械臂运动计划..."
            )

            motion_plan = self.create_motion_plan()

            if motion_plan is None:
                self.report_error(
                    "运动计划生成失败，任务停止"
                )
                return

            completed = 0

            for cycle in range(
                1,
                self.task_repeat_limit + 1,
            ):
                if (
                    self.alternate_direction
                    and cycle % 2 == 0
                ):
                    source = "dropoff"
                    destination = "pickup"
                    direction_text = "B→A"
                else:
                    source = "pickup"
                    destination = "dropoff"
                    direction_text = "A→B"

                self.publish_status(
                    f"========== 第 {cycle}/"
                    f"{self.task_repeat_limit} 次："
                    f"{direction_text} =========="
                )

                success = self._execute_cycle(
                    cycle,
                    motion_plan,
                    source,
                    destination,
                )

                if success:
                    completed += 1
                else:
                    self.publish_status(
                        "当前轮次失败，机械臂返回安全位置"
                    )
                    self.sim_object.stop()
                    self._move_home(motion_plan)

                    if self.alternate_direction:
                        break

                if cycle < self.task_repeat_limit:
                    if self.alternate_direction and success:
                        # 往返模式下，上一轮的终点就是下一轮的起点。
                        pass
                    else:
                        next_source = (
                            "dropoff"
                            if (
                                self.alternate_direction
                                and cycle % 2 == 0
                            )
                            else "pickup"
                        )

                        if not self._reset_for_next_cycle(
                            next_source
                        ):
                            break

            summary = (
                f"任务结束：计划执行 "
                f"{self.task_repeat_limit} 次，"
                f"实际完成 {completed} 次"
            )

            self.publish_status(summary)

            with open(
                os.path.join(
                    self.log_dir,
                    "summary.txt",
                ),
                "w",
            ) as summary_file:
                summary_file.write(
                    summary + "\n"
                )

        except Exception as exc:
            self.report_error(
                f"任务线程发生未处理异常：{exc}"
            )

        finally:
            self.sim_object.stop()

            if not self.trajectory_log.closed:
                self.trajectory_log.close()

            if not self.result_log.closed:
                self.result_log.close()

            if not self.error_log.closed:
                self.error_log.close()

            self.get_logger().info(
                "抓取任务节点结束"
            )

            rclpy.try_shutdown()


def main():
    rclpy.init()

    node = TransferController()

    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
