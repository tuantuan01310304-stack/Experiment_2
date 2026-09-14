#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    'joint1_to_base',
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
]

GRIPPER_JOINT = 'gripper_controller'


# 昨天真机实际验证成功的四个点，单位：degree
POINTS_DEG = {
    'HOME':    [-6.76, 43.94, 2.02, -1.75, 58.44, -0.08],
    'A_SAFE':  [-9.49, 42.62, 6.06, -6.15, 7.82, 2.46],
    'A_CLEAR': [-9.49, 38.00, 7.47, -6.15, 7.82, 2.46],
    'A_PICK':  [-9.49, 48.86, 5.36, -6.15, 7.82, 2.46],
    'B_CLEAR': [25.00, 38.00, 7.47, -6.15, 7.82, 2.46],
    'B_PLACE': [25.00, 48.51, 7.47, -6.15, 7.82, 2.46],
}


def deg2rad(values):
    return [math.radians(v) for v in values]


class RealTransferLWS(Node):

    def __init__(self):
        super().__init__('real_transfer_lws')

        # 默认禁止运动。必须显式 execute:=true 才允许发送动作。
        self.declare_parameter('execute', False)
        self.declare_parameter('move_duration', 5.0)
        self.declare_parameter('grip_duration', 2.0)

        self.execute = bool(self.get_parameter('execute').value)
        self.move_duration = float(
            self.get_parameter('move_duration').value
        )
        self.grip_duration = float(
            self.get_parameter('grip_duration').value
        )

        self.arm_cli = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.hand_cli = ActionClient(
            self,
            FollowJointTrajectory,
            '/hand_controller/follow_joint_trajectory'
        )

        self.current_joints = None

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self.points = {
            name: deg2rad(value)
            for name, value in POINTS_DEG.items()
        }

    def _joint_state_cb(self, msg):
        table = dict(zip(msg.name, msg.position))

        if all(j in table for j in ARM_JOINTS):
            self.current_joints = [
                float(table[j]) for j in ARM_JOINTS
            ]

    def send_goal(self, client, joints, positions, duration):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joints)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(
            (duration - int(duration)) * 1e9
        )

        goal.trajectory.points = [point]

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if handle is None or not handle.accepted:
            self.get_logger().error('Action 目标被拒绝')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f'Action 执行失败: {result.error_string}'
            )
            return False

        return True

    def move_arm(self, point_name):
        self.get_logger().info(
            f'移动到 {point_name}: {POINTS_DEG[point_name]} deg'
        )

        return self.send_goal(
            self.arm_cli,
            ARM_JOINTS,
            self.points[point_name],
            self.move_duration
        )

    def open_gripper(self):
        self.get_logger().info('打开夹爪')
        return self.send_goal(
            self.hand_cli,
            [GRIPPER_JOINT],
            [0.0],
            self.grip_duration
        )

    def close_gripper(self):
        self.get_logger().info('闭合夹爪')
        return self.send_goal(
            self.hand_cli,
            [GRIPPER_JOINT],
            [-0.44],
            self.grip_duration
        )

    def run(self):

        self.get_logger().info('===== LWS 真机 ROS2 连续5段 A -> B -> A -> B -> A -> B =====')

        for name in ['HOME', 'A_SAFE', 'A_CLEAR', 'A_PICK', 'B_CLEAR', 'B_PLACE']:
            self.get_logger().info(
                f'{name} = {POINTS_DEG[name]} deg'
            )

        if not self.execute:
            self.get_logger().warn(
                'DRY-RUN 模式：execute=false，不会发送任何运动指令'
            )
            return

        self.get_logger().warn(
            'execute=true：准备执行真实机械臂动作'
        )

        if not self.arm_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('找不到机械臂 Action Server')
            return

        if not self.hand_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('找不到夹爪 Action Server')
            return

        # 先确认能收到真实机械臂状态
        t0 = time.time()
        while self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - t0 > 10.0:
                self.get_logger().error(
                    '10 秒内没有收到 /joint_states，停止'
                )
                return

        current_deg = [
            round(math.degrees(v), 2)
            for v in self.current_joints
        ]

        self.get_logger().info(
            f'当前机械臂姿态 = {current_deg} deg'
        )

        # 连续5段搬运，每完成一段均返回独立 HOME
        steps = [
            # 第1段：A -> B -> HOME
            ('HOME', self.open_gripper),
            ('A_SAFE', None),
            ('A_PICK', self.close_gripper),
            ('A_CLEAR', None),
            ('B_CLEAR', None),
            ('B_PLACE', self.open_gripper),
            ('B_CLEAR', None),
            ('HOME', None),

            # 第2段：B -> A -> HOME
            ('B_CLEAR', None),
            ('B_PLACE', self.close_gripper),
            ('B_CLEAR', None),
            ('A_CLEAR', None),
            ('A_PICK', self.open_gripper),
            ('A_CLEAR', None),
            ('HOME', None),

            # 第3段：A -> B -> HOME
            ('A_SAFE', None),
            ('A_PICK', self.close_gripper),
            ('A_CLEAR', None),
            ('B_CLEAR', None),
            ('B_PLACE', self.open_gripper),
            ('B_CLEAR', None),
            ('HOME', None),

            # 第4段：B -> A -> HOME
            ('B_CLEAR', None),
            ('B_PLACE', self.close_gripper),
            ('B_CLEAR', None),
            ('A_CLEAR', None),
            ('A_PICK', self.open_gripper),
            ('A_CLEAR', None),
            ('HOME', None),

            # 第5段：A -> B -> HOME
            ('A_SAFE', None),
            ('A_PICK', self.close_gripper),
            ('A_CLEAR', None),
            ('B_CLEAR', None),
            ('B_PLACE', self.open_gripper),
            ('B_CLEAR', None),
            ('HOME', None),
        ]

        for point_name, gripper_action in steps:

            if not self.move_arm(point_name):
                self.get_logger().error(
                    f'在 {point_name} 失败，立即停止后续动作'
                )
                return

            time.sleep(0.2)

            if gripper_action is not None:
                if not gripper_action():
                    self.get_logger().error(
                        '夹爪动作失败，立即停止后续动作'
                    )
                    return

                time.sleep(0.3)

        self.get_logger().info(
            '===== ROS2 连续5段搬运完成 ====='
        )


def main(args=None):
    rclpy.init(args=args)

    node = RealTransferLWS()

    try:
        node.run()
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
