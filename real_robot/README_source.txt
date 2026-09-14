机械臂定点抓取实验——实验报告材料包

1. code_grasp/
   ROS2抓取任务代码、最终连续五轮成功代码、package配置、
   controllers/launch等实验配置。

2. code_driver/
   真机real_driver、TCP客户端、关节限位和real.yaml参数。

3. logs/
   连续五轮真机运行记录、历史5次测试记录、
   关节超限异常测试记录。

最终推荐代码：
real_cycle_5_home_fast3_FINAL_SUCCESS.py

最终HOME：
[-6.76, 43.94, 2.02, -1.75, 58.44, -0.08] deg

异常测试：
向joint1_to_base发送170°超限目标，
系统返回ABORTED，测试前后joint_states保持不变。
