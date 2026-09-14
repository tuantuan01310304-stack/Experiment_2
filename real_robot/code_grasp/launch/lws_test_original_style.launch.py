"""一键启动完整仿真系统（验收要求：单一 Launch 文件）。

启动内容：Gazebo Fortress 世界 → 机器人生成 → ros2_control 控制器
→ 状态发布(TF/joint_states) → 抓取任务节点。
参数：
  gui:=false   无头运行（开发调试/CI 用，桌面里默认开 GUI）
  cycles:=N    连续抓取次数（默认取参数文件里的 5）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('mecharm_grasp_lws')
    # world:=grasp_world_noobject.sdf 可切换到"无目标物"场景做异常测试
    world = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])
    xacro_file = os.path.join(pkg, 'urdf', 'mecharm_270_gazebo.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'controllers.yaml')
    # params:=grasp_params_unreachable.yaml 可切换到"不可达目标"参数做异常测试
    params = PathJoinSubstitution([pkg, 'config', LaunchConfiguration('params')])

    gui = LaunchConfiguration('gui')
    # 网格用 package:// 引用，把相关包的 install/share 都加进 Gazebo 资源
    # 搜索路径（colcon 隔离安装：每个包各有自己的 share 根，缺一不可）
    desc_pkg = get_package_share_directory('mycobot_description')
    share_roots = [os.path.dirname(pkg), os.path.dirname(desc_pkg)]
    resource_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join(share_roots + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers]),
        value_type=str)

    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen', condition=IfCondition(gui))
    gz_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world],
        output='screen', condition=UnlessCondition(gui))

    # 桥接：时钟 + 目标物的速度指令(ROS→GZ)与里程计(GZ→ROS)，吸附搬运用
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/target_object/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/target_object/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    # 机器人生成在桌面上（z=0.75 与 lws_grasp_params.yaml 的 base_pose_xyz 一致）
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mecharm',
                   '-x', '0', '-y', '0', '-z', '0.75'],
        output='screen')

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '60'],
                    output='screen')

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    hand = spawner('hand_controller')

    grasp = Node(
        package='mecharm_grasp_lws', executable='grasp_task_lws',
        parameters=[params], output='screen',
        condition=IfCondition(LaunchConfiguration('task')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        # task:=false 只起仿真不跑任务（标定/调试用）
        DeclareLaunchArgument('task', default_value='true'),
        DeclareLaunchArgument('world', default_value='lws_grasp_world.sdf'),
        DeclareLaunchArgument('params', default_value='lws_grasp_params.yaml'),
        resource_env,
        gz_gui, gz_headless,
        clock_bridge, rsp, spawn,
        # 机器人生成完成后依次拉起控制器，最后启动任务节点
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, hand])),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grasp])),
    ])
