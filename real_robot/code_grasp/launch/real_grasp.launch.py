"""真机（Jetson + mechArm 270）定点抓取一键启动。

启动：真机驱动节点（pymycobot）+ 任务节点（与仿真共用）。
用法：
  ros2 launch mecharm_grasp_lws real_grasp.launch.py                 # 跑 5 次抓取
  ros2 launch mecharm_grasp_lws real_grasp.launch.py cycles:=1      # 先单次低速验证
  急停：硬件急停优先；软件急停另开终端
  ros2 topic pub --once /soft_stop std_msgs/msg/Bool "data: true"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mecharm_grasp_lws')
    params = os.path.join(share, 'config', 'real_grasp_params.yaml')
    cycles = LaunchConfiguration('cycles')

    driver = Node(package='mecharm_grasp_lws', executable='real_driver',
                  output='screen', parameters=[params])
    task = Node(package='mecharm_grasp_lws', executable='grasp_task',
                output='screen', parameters=[
                    params,
                    {'cycles': ParameterValue(cycles, value_type=int)},
                ])

    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='5'),
        driver,
        # 驱动先起并完成串口连接，任务节点延后启动
        TimerAction(period=4.0, actions=[task]),
    ])
