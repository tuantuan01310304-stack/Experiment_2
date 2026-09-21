#!/usr/bin/env python3
"""LWS 机械臂双工位往返搬运程序。

任务顺序保持为：初始位 → 取物点上方 → 下探夹取 → 抬升 → 放置点上方
→ 下探释放 → 返回初始位。A、B 两个工位以及运动相关参数统一从 ROS2
参数中读取，因此同一套任务流程可以用于 Gazebo 仿真和真实硬件环境。

程序主要包含以下部分：
1. 使用数值法完成机械臂逆运动学求解，并在下发前检查关节范围；
2. 通过 FollowJointTrajectory 控制机械臂和夹爪执行连续动作；
3. 仿真模式下让目标物体平滑跟随夹持中心，真机模式可关闭该功能；
4. 将轨迹、执行结果和异常信息分别保存到日志文件，便于任务检查。
"""
import math
import os
import subprocess
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ARM_JOINTS = [
    'joint1_to_base', 'joint2_to_joint1', 'joint3_to_joint2',
    'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5',
]
GRIPPER_JOINT = 'gripper_controller'

# 六个机械臂关节的允许角度范围，单位为 rad
JOINT_LIMITS = [
    (-2.792527, 2.792527), (-1.3089, 2.0943), (-3.0543, 1.1344),
    (-2.7052, 2.7052), (-2.0071, 2.0071), (-3.14, 3.14),
]

# ---------- 机械臂运动学计算 ----------

def rpy_mat(r, p, y):#roll/pitch/yaw  得出旋转矩阵
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T#构造齐次变化矩阵


# 按 URDF 中的关节原点建立连杆变换，随后叠加各关节的 z 轴转动
CHAIN = [
    make_T([0, 0, 0.1], [0, 0, 0]),
    make_T([0, 0, 0.038], [-1.5708, 0, 0]),
    make_T([0.0, -0.1, 0], [0, 0, 0]),
    make_T([0.108, -0.005, -0.001], [0, 1.5708, 0]),
    make_T([-0.001, 0, 0.0], [0, -1.5708, 0]),
    make_T([0.06, 0.0, -0.0], [0, 1.5708, 0]),#构造机械臂连杆模型，一个关节相对于前一个关节的位移与旋转
]
T_FLANGE = make_T([0, 0, 0.038], [1.579, 0, 0])  # link6 → gripper_base


def rz(q):#rotate z
    T = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


def fk(q):#正运动学，在base坐标系中的位置
    """返回 (link6 位姿 4x4, gripper_base 位姿 4x4)，基座坐标系。"""
    T = np.eye(4)#base坐标系
    for i in range(6):
        T = T @ CHAIN[i] @ rz(q[i])
    return T, T @ T_FLANGE


def tip_pos(q, tool_offset):#末端到达目标点
    """指尖位置（基座系）：gripper_base 原点沿 link6 z 轴（工具接近轴）前伸。"""
    T6, Tg = fk(q)
    return Tg[:3, 3] + T6[:3, 2] * tool_offset, T6[:3, 2]#gripper_base的位置+工具轴方向*工具长度

#逆运动学：通过目标位置去推测如何到达
def solve_ik(target, tool_offset, seed=None, iters=250):
    """数值 IK：指尖到 target，工具轴接近竖直向下（允许 ≤ ~15° 倾角，
    小臂行程有限，严格竖直会把大量桌面区域判为不可达）。
    位置权重高于姿态。返回 q 或 None（不可达）。"""
    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])
    seeds = [seed] if seed is not None else []#初始猜测，如果上一个位置游街，就直接优先拿它作为下一次IK的初始值
    yaw = math.atan2(target[1], target[0])#算yaw
    seeds += [
        np.array([yaw, 0.8, -1.6, 0.0, -0.8, 0.0]),
        np.array([yaw, 0.4, -1.0, 0.0, -1.0, 0.0]),
        np.array([yaw, 1.2, -2.0, 0.0, -0.6, 0.0]),
    ]
    rng = np.random.default_rng(42)  # 固定种子保证可复现
    seeds += [lo + rng.random(6) * (hi - lo) for _ in range(12)]#关节角
    down = np.array([0, 0, -1.0])#工具轴设置为世界坐标的负方向，也就是夹爪竖直向下
    AXIS_W = 0.4  # 姿态项权重（位置为 1）
    # IK 优先选择接近参考姿态的解，减少连续动作之间的大幅关节跳变
    q_ref = seed if seed is not None else np.array([yaw, 0.6, -1.2, 0.0, -0.8, 0.0])
    solutions = []
    for q in seeds:#开始尝试每一个初始值
        q = np.clip(np.asarray(q, dtype=float).copy(), lo, hi)
        for _ in range(iters):
            p, axis = tip_pos(q, tool_offset)#算当前之间位置p工具方向axis
            r = np.concatenate([p - target, AXIS_W * (axis - down)])#计算误差（包括位置误差和姿态误差）
            if np.linalg.norm(p - target) < 0.002 and np.linalg.norm(axis - down) < 0.26:#判断IK是否成功，成功保存
                solutions.append(np.clip(q, lo, hi))
                break
            J = np.zeros((6, 6))#不成功计算雅可比矩阵
            eps = 1e-5#增加微小扰动
            for i in range(6):#六个关节都一点一点的动
                dq = q.copy()
                dq[i] += eps
                p2, a2 = tip_pos(dq, tool_offset)
                J[:, i] = np.concatenate([(p2 - p), AXIS_W * (a2 - axis)]) / eps#用雅可比矩阵计算每一列队末端误差的影响
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ r)#阻尼最小二乘IK更新量，是数值计算更稳定
            q = np.clip(q - np.clip(step, -0.3, 0.3), lo, hi)#限制每次关节变化
    if not solutions:
        return None
    return min(solutions, key=lambda s: np.linalg.norm(s - q_ref))#如果找到多个解，就选择里参考姿态最近的解


def align_jaw(q):#解决机械臂到目标位置但是夹爪方向歪了
    """调整腕转角 q6，使夹爪开合方向（gripper_base x 轴）对齐世界坐标轴。
    方块的面沿世界轴摆放，虎口若斜着合拢会怼在棱角上把物体挤出去。
    数值牛顿迭代：q6 对水平开合角的导数用有限差分求。"""
    q = q.copy()
    for _ in range(4):
        _, Tg = fk(q)
        jaw = Tg[:3, 0]#取gripperbase的x轴方向
        phi = math.atan2(jaw[1], jaw[0])#夹爪开合方向在XY平面的角度
        delta = phi - round(phi / (math.pi / 2)) * (math.pi / 2)#看当前的夹爪方向距离最近的教务有多远
        if abs(delta) < 0.01:#如果非常接近就不用调整
            break
        q2 = q.copy()
        q2[5] += 0.01
        _, Tg2 = fk(q2)
        jaw2 = Tg2[:3, 0]#调整q6看夹爪方向的变化
        dphi = (math.atan2(jaw2[1], jaw2[0]) - phi) / 0.01
        if abs(dphi) < 1e-3:
            break
        q[5] = np.clip(q[5] - delta / dphi, JOINT_LIMITS[5][0], JOINT_LIMITS[5][1])
    return q


class LwsTransferTask(Node):#正式进入任务
    def __init__(self):
        super().__init__('lws_transfer_node')
        for name, default in [#写入参数
            ('base_pose_xyz', [0.0, 0.0, 0.75]), ('station_a_position', [0.10, 0.02, 0.7635]),
            ('station_b_position', [0.07, -0.08, 0.7635]), ('safe_height', 0.08),
            ('station_a_offset', 0.002), ('station_b_offset', 0.008),
            ('placement_tolerance', 0.03),
            ('tool_tip_offset', 0.10), ('gripper_open', 0.1), ('gripper_close', -0.55),
            ('arm_move_time', 2.55), ('gripper_move_time', 1.25), ('repeat_count', 5),
            ('log_dir', '/home/nvidia/lws/grasp_logs'), ('world_name', 'grasp_world'),
            ('sim_attach', True), ('sim_check', True),
        ]:
            self.declare_parameter(name, default)
        read_param = lambda n: self.get_parameter(n).value#读取参数，把参数保存为对象成员
        self.base = np.array(read_param('base_pose_xyz'))
        self.station_a_position = np.array(read_param('station_a_position'))
        self.station_b_position = np.array(read_param('station_b_position'))
        self.safe_h = read_param('safe_height')
        self.station_a_offset = read_param('station_a_offset')
        self.station_b_offset = read_param('station_b_offset')
        self.placement_tolerance = read_param('placement_tolerance')
        self.tool_off = read_param('tool_tip_offset')
        self.grip_open, self.grip_close = read_param('gripper_open'), read_param('gripper_close')
        self.arm_move_time, self.gripper_move_time = read_param('arm_move_time'), read_param('gripper_move_time')
        self.repeat_count = int(read_param('repeat_count'))
        self.world = read_param('world_name')
        self.sim_attach = bool(read_param('sim_attach'))
        # 仿真模式读取 Gazebo 物体位姿用于结果检查；真机模式关闭该功能
        self.sim_check = bool(read_param('sim_check'))
        self.log_dir = os.path.expanduser(read_param('log_dir'))
        os.makedirs(self.log_dir, exist_ok=True)

        callback_group = ReentrantCallbackGroup()
        #创建机械臂Action Client
        self.arm_cli = ActionClient(self, FollowJointTrajectory,#机械臂轨迹
                                    '/arm_controller/follow_joint_trajectory', callback_group=callback_group)
        self.hand_cli = ActionClient(self, FollowJointTrajectory,#夹爪
                                     '/hand_controller/follow_joint_trajectory', callback_group=callback_group)
        self.status_pub = self.create_publisher(String, '/grasp_status', 10)#状态发布器
        self.joint_q = {}#订阅关节状态，监听/joint_states，每次收到关节状态self._on_js()就会被调用
        self.create_subscription(JointState, '/joint_states', self._on_js, 10, callback_group=callback_group)
        # 吸附搬运：物体里程计反馈 + 速度指令（经 ros_gz_bridge 常驻桥接，30Hz 平滑）
        self.obj_pos = None#物体位置
        self.create_subscription(Odometry, '/model/target_object/odometry',
                                 self._on_obj_odom, 10, callback_group=callback_group)#订阅里程计
        self.obj_vel_pub = self.create_publisher(Twist, '/model/target_object/cmd_vel', 10)#控制物体速度，Gazebo 中的物体实际上通过速度控制跟随机械臂
#创建日志文件、轨迹记录、错误记录每次搬运的最终位置等
        self.traj_file = open(os.path.join(self.log_dir, 'trajectory.csv'), 'w')
        self.traj_file.write('stamp,' + ','.join(ARM_JOINTS + [GRIPPER_JOINT]) + '\n')
        self.err_file = open(os.path.join(self.log_dir, 'errors.log'), 'a')
        self.res_file = open(os.path.join(self.log_dir, 'results.csv'), 'w')
        self.res_file.write('cycle,success,object_x,object_y,object_z,detail\n')
#吸附定时器，等待0.5秒
        self.attach_on = False
        self.attach_timer = self.create_timer(1.0 / 30, self._attach_tick, callback_group=callback_group)
        #启动任务线程，节点初始化完成后自动开始搬运任务
        self.worker = threading.Thread(target=self.run_task, daemon=True)
        self.worker.start()

    # ---------- ROS2 通信与运行记录 ----------
    def status(self, text):
        self.get_logger().info(text)#打印
        self.status_pub.publish(String(data=text))#发布
    #处理错误+保存错误日志
    def fail(self, text):
        self.get_logger().error(text)
        self.err_file.write(f'{datetime.now().isoformat()} {text}\n')
        self.err_file.flush()
        self.status_pub.publish(String(data='ERROR: ' + text))
   #读取joint_states
    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_q[n] = p
        if self.traj_file and not self.traj_file.closed:
            row = [f'{self.joint_q.get(j, float("nan")):.4f}' for j in ARM_JOINTS + [GRIPPER_JOINT]]
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.traj_file.write(f'{t:.3f},' + ','.join(row) + '\n')

    # ---------- 机械臂 / 夹爪轨迹控制 ----------
    def _send_traj(self, client, joints, positions, duration):#发送机械臂动作
        for j, p in zip(joints, positions):#逐个检查关节
            if j in ARM_JOINTS:
                lo, hi = JOINT_LIMITS[ARM_JOINTS.index(j)]
                if not (lo - 1e-6 <= p <= hi + 1e-6):
                    self.fail(f'关节 {j} 目标 {p:.3f} 超限 [{lo:.3f},{hi:.3f}]，拒绝执行')
                    return False
        #构造轨迹
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joints)#要控制哪些关节
        pt = JointTrajectoryPoint()#创建轨迹点
        pt.positions = [float(p) for p in positions]#设置目标
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        #发送
        fut = client.send_goal_async(goal)
        wait_start = time.time()
        while not fut.done():#等待回应
            time.sleep(0.05)
            if time.time() - wait_start > 10:
                self.fail('发送轨迹目标超时（通信异常）')
                return False
        gh = fut.result()
        if not gh.accepted:
            self.fail('控制器拒绝轨迹目标')
            return False
        rf = gh.get_result_async()
        wait_start = time.time()
        # Gazebo 图形界面运行较慢时，实际等待时间可能明显长于轨迹设定时间。
        # 因此根据动作时长动态放宽超时阈值，避免正常的慢速动作被误判为失败。
        while not rf.done():
            time.sleep(0.05)
            if time.time() - wait_start > duration * 10 + 60:
                self.fail('轨迹执行超时')
                return False
        res = rf.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.fail(f'轨迹执行失败 error_code={res.error_code} {res.error_string}')
            return False
        return True

    def move_arm(self, q, duration=None):#简单封装，控制机械臂六个关节
        return self._send_traj(self.arm_cli, ARM_JOINTS, q, duration or self.arm_move_time)

    def move_gripper(self, pos):#控制夹爪
        return self._send_traj(self.hand_cli, [GRIPPER_JOINT], [pos], self.gripper_move_time)

    # ---------- Gazebo 目标物体跟随 ----------
    def _set_object_pose(self, xyz):
        req = (f'name: "target_object", position: {{x: {xyz[0]:.4f}, '
               f'y: {xyz[1]:.4f}, z: {xyz[2]:.4f}}}')
        subprocess.run(
            ['ign', 'service', '-s', f'/world/{self.world}/set_pose',
             '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '200', '--req', req],
            capture_output=True, timeout=2)

    def _on_obj_odom(self, msg):
        p = msg.pose.pose.position
        self.obj_pos = np.array([p.x, p.y, p.z])
#吸附
    def _attach_tick(self):
        """30Hz 速度伺服：物体平滑追随夹持中心（P 控制，限速），
        比逐次调 set_pose 服务的"瞬移"流畅得多。"""
        if not (self.attach_on and self.sim_attach) or self.obj_pos is None:
            return#吸附开始、仿真模式、知道物体位置
        # 释放阶段：伺服目标切换为 B 点本身，把物体精确送到位再松手
        if getattr(self, 'place_target', None) is not None:
            target = self.place_target#如果正在放置，就往B点和A点收敛
        else:#否则计算当前位置在世界坐标系的位置
            q = [self.joint_q.get(j) for j in ARM_JOINTS]#构造p控制器，发布物体速度，让gazebo物体超目标位置移动，跟着夹爪走
            if any(v is None for v in q):
                return
            p, _ = tip_pos(np.array(q), self.tool_off)
            target = self.base + p
        v = np.clip(6.0 * (target - self.obj_pos), -0.6, 0.6)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = float(v[0]), float(v[1]), float(v[2])
        self.obj_vel_pub.publish(cmd)

    def object_world_pose(self):
        """读取物体真值位姿，用于成功判定（仿真专用）。"""
        try:
            out = subprocess.run(
                ['ign', 'topic', '-e', '-n', '1', '-t', f'/world/{self.world}/pose/info'],
                capture_output=True, text=True, timeout=3).stdout
            blocks = out.split('pose {')
            for b in blocks:
                if '"target_object"' in b:
                    import re
                    m = re.search(r'position\s*{\s*x:\s*([-\d.e]+)\s*y:\s*([-\d.e]+)\s*z:\s*([-\d.e]+)', b)
                    if m:
                        return np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))])
        except Exception as e:
            self.fail(f'读取物体位姿失败: {e}')
        return None

    # ---------- A/B 工位往返流程 ----------
    def build_transfer_waypoints(self):#构造四个关键位置
        """计算 A、B 两个工作点对应的关节目标；存在不可达位置时返回 None。"""
        point_a_local = self.station_a_position - self.base
        point_b_local = self.station_b_position - self.base
        waypoints = {}

        # A、B 点分别设置接近位置和实际操作位置。
        # 上方位置用于安全转移，较低位置用于夹取或放置。
        pose_targets = {
            'above_a': point_a_local + [0, 0, self.safe_h],
            'at_a': point_a_local + [0, 0, self.station_a_offset],
            'above_b': point_b_local + [0, 0, self.safe_h],
            'at_b': point_b_local + [0, 0, self.station_b_offset],
        }

        seed = None
        for name, target_position in pose_targets.items():
            reach = np.linalg.norm(
                target_position - np.array([0, 0, 0.138])
            )
            if reach > 0.30:
                self.fail(
                    f'目标位置 {name} 超出机械臂工作范围 '
                    f'({reach:.3f}m > 0.30m)'
                )
                return None
            #对四个位置逐个做IK
            ik_solution = solve_ik(
                np.array(target_position),
                self.tool_off,
                seed=seed
            )
            if ik_solution is None:
                self.fail(f'目标位置 {name} 无法获得有效关节解，停止当前任务')
                return None
            #夹爪位置修正
            ik_solution = align_jaw(ik_solution)
            waypoints[name] = ik_solution
            seed = ik_solution

        waypoints['home'] = np.zeros(6)#作为初始姿态q1=q2=q3=q4=q5=q6=0
        return waypoints

    def run_task(self):
        try:
            self._run_task_impl()
        finally:
            # 任务结束（含出错）后主动退出进程：残留的旧节点会与新实例
            # 抢同一个控制器（曾出现双 action server 乱象），必须自杀清场
            self.get_logger().info('任务节点退出')
            rclpy.try_shutdown()

    def _abort(self, reason):
        """异常终止：错误日志 + 结果摘要都留痕（验收要求"明确错误信息"），机械臂不动。"""
        self.fail(reason)
        summary = f'任务终止（异常安全停止）：{reason}'
        self.status(summary)
        with open(os.path.join(self.log_dir, 'summary.txt'), 'w') as f:
            f.write(summary + '\n')
        self.res_file.write(f'-,0,,,,异常终止:{reason}\n')
        self.res_file.flush()
        self.traj_file.close()
        self.res_file.close()

    def _run_task_impl(self):#主流程
        time.sleep(2.0)
        self.status('正在连接机械臂控制器...')
        if not (self.arm_cli.wait_for_server(timeout_sec=60) and self.hand_cli.wait_for_server(timeout_sec=60)):
            self.fail('控制器动作服务器未上线，任务终止')
            return
        wait_start = time.time()
        while not all(j in self.joint_q for j in ARM_JOINTS):
            time.sleep(0.2)
            if time.time() - wait_start > 30:
                self.fail('等不到 /joint_states，通信异常，任务终止')
                return

        self.status('正在计算各目标位姿...')
        joint_waypoints = self.build_transfer_waypoints()
        if joint_waypoints is None:
            self._abort('目标不可达（超出工作半径或无逆解），任务拒绝启动，机械臂保持安全停止')
            return

        # 仿真任务开始前先确认目标模型存在，避免机械臂空执行
        if self.sim_check:
            self.status('正在确认目标物体位置...')
            pos = self.object_world_pose()
            if pos is None:
                self._abort('场景中未检测到目标物体 target_object，任务终止，机械臂保持安全停止')
                return
            self.status(f'已获取目标物体位置: {np.round(pos, 4)}')

        success_count = 0
        for cycle in range(1, self.repeat_count + 1):
            # 往返接力：单数次 A→B，双数次 B→A。物体不做任何"瞬移复位"，
            # 上一次的放置点就是下一次的取物点，全程物理连续。
            pick, place = ('a', 'b') if cycle % 2 == 1 else ('b', 'a')
            self.status(f'===== 第 {cycle}/{self.repeat_count} 轮搬运任务（{pick.upper()}→{place.upper()}）=====')
            ok = self.execute_transfer_cycle(cycle, joint_waypoints, pick, place)
            if ok:
                success_count += 1
            else:
                self.status('当前轮次未完成，机械臂撤回')
                self.attach_on = False
                self.move_arm(joint_waypoints['home'])
                break  # 往返链条断了，后续方向对不上，安全停止
        summary = f'搬运流程结束：共执行 {self.repeat_count} 轮，完成 {success_count} 轮'
        self.status(summary)
        with open(os.path.join(self.log_dir, 'summary.txt'), 'w') as f:
            f.write(summary + '\n')
        self.traj_file.close()
        self.res_file.close()

    def _reset_object(self):
        if self.sim_check:
            for _ in range(4):
                try:
                    self._set_object_pose(self.station_a_position)
                except Exception:
                    pass
                time.sleep(0.5)
                if (self.obj_pos is not None
                        and np.linalg.norm(self.obj_pos[:2] - self.station_a_position[:2]) < 0.01):
                    return
            self.fail('物体复位到 A 点失败')

    def run_transfer_step(self, cycle, step_name, action, pause_time):
        """执行单个搬运动作，并统一处理状态输出、失败记录和动作间停顿。"""
        self.status(f'[{cycle}] {step_name}')

        if not action():
            self.res_file.write(
                f'{cycle},0,,,,失败于步骤:{step_name}\n'
            )
            self.res_file.flush()
            return False

        time.sleep(pause_time)
        return True

    def prepare_transfer(self, cycle, joint_waypoints):
        """搬运开始前回到初始姿态，并将夹爪调整为准备状态。"""#初始姿态：回零和张开夹爪
        if not self.run_transfer_step(
            cycle,
            '返回初始姿态',
            lambda: self.move_arm(joint_waypoints['home']),
            0.8
        ):
            return False

        if not self.run_transfer_step(
            cycle,
            '夹爪准备',
            lambda: self.move_gripper(self.grip_open),
            0.6
        ):
            return False

        return True

    def pick_object(self, cycle, joint_waypoints, pick):
        """完成取物点接近、夹取和提升动作。"""
        source_label = pick.upper()

        pick_steps = [
            (
                f'移动至{source_label}点等待位置',
                lambda: self.move_arm(joint_waypoints[f'above_{pick}']),
                1.0
            ),
            (
                '靠近目标物体',
                lambda: self.move_arm(
                    joint_waypoints[f'at_{pick}'],#下降
                    self.arm_move_time * 0.6
                ),
                0.8
            ),
            (
                '闭合夹爪',
                self.close_gripper_for_pick,
                1.5
            ),
            (
                '提升目标物体',
                lambda: self.move_arm(
                    joint_waypoints[f'above_{pick}'],
                    self.arm_move_time * 0.6
                ),
                0.8
            ),
        ]

        for step_name, action, pause_time in pick_steps:
            if not self.run_transfer_step(
                cycle, step_name, action, pause_time
            ):
                return False

        return True

    def place_object(self, cycle, joint_waypoints, place):
        """将物体转移到目标点，完成释放并安全离开目标区域。"""
        destination_label = place.upper()

        place_steps = [
            (
                f'转移至{destination_label}点等待位置',
                lambda: self.move_arm(joint_waypoints[f'above_{place}']),
                1.0
            ),
            (
                '靠近目标位置',
                lambda: self.move_arm(
                    joint_waypoints[f'at_{place}'],
                    self.arm_move_time * 0.6
                ),
                0.8
            ),
            (
                '松开夹爪',
                lambda: self.open_gripper_for_place(place),
                1.5
            ),
            (
                '离开目标区域',
                lambda: self.move_arm(
                    joint_waypoints[f'above_{place}'],
                    self.arm_move_time * 0.6
                ),
                0.8
            ),
        ]

        for step_name, action, pause_time in place_steps:
            if not self.run_transfer_step(
                cycle, step_name, action, pause_time
            ):
                return False

        return True

    def execute_transfer_cycle(self, cycle, joint_waypoints, pick='a', place='b'):#把前面的所有动作贯穿起来
        # 每轮的起点和终点由主循环指定，因此同一流程可双向复用
        if not self.prepare_transfer(cycle, joint_waypoints):
            return False

        if not self.pick_object(cycle, joint_waypoints, pick):
            return False

        if not self.place_object(cycle, joint_waypoints, place):
            return False

        if not self.run_transfer_step(
            cycle,
            '返回初始姿态',
            lambda: self.move_arm(joint_waypoints['home']),
            0.8
        ):
            return False

        if not self.sim_check:
            # 真机：没有物体位姿真值，全部步骤走完即计成功，落点由现场人工核对
            self.res_file.write(f'{cycle},1,,,,完成（落点需人工确认）\n')
            self.res_file.flush()
            return True
        pos = self.object_world_pose()
        target = self.station_b_position if place == 'b' else self.station_a_position
        placed = pos is not None and np.linalg.norm(pos[:2] - target[:2]) < self.placement_tolerance
        detail = 'ok' if placed else '物体不在放置区'
        self.res_file.write(f'{cycle},{1 if placed else 0},'
                            + (f'{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}' if pos is not None else ',,')
                            + f',{detail}\n')
        self.res_file.flush()
        return placed

    def close_gripper_for_pick(self):
        # 顺序：先吸附定住物体，再闭合到"环抱不接触"的角度（笼抱式）。
        # 手指与物体全程零接触——接触力会与吸附伺服互相对抗，
        # 之前导致手指陷入物体、抓取过程抖动。真机上闭合角映射为
        # 力控自适应夹爪的行程指令，接触由夹爪固件自己处理。
        if self.sim_attach:
            self.attach_on = True
            time.sleep(0.4)  # 等吸附伺服把物体定到工具中心
        ok = self.move_gripper(self.grip_close)
        if not ok:
            self.attach_on = False
            return False
        if not self.sim_attach:
            self.attach_on = True
        return True

    def open_gripper_for_place(self, place='b'):
        # 顺序：物体已随手臂降到桌面高度（at_* 工具目标只比静置中心高 2mm）
        # → 静置片刻让吸附伺服把它收敛到放置点正上方 → 停止吸附（物体已
        # 贴桌面，不存在下落过程）→ 最后才张开夹爪。
        # 之前"先开爪、再伺服拖到放置点"的顺序会出现物体悬空滑移的粘滞感。
        if self.sim_attach:
            time.sleep(0.8)  # 收敛：伺服增益 6/s，0.8s 后残差 < 1mm
            self.attach_on = False
            # VelocityControl 会一直执行最后一条速度指令，必须归零，
            # 否则物体在释放后仍带着残余速度漂走
            for _ in range(3):
                self.obj_vel_pub.publish(Twist())
                time.sleep(0.05)
        else:
            self.attach_on = False
        return self.move_gripper(self.grip_open)


def main():
    rclpy.init()
    node = LwsTransferTask()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
