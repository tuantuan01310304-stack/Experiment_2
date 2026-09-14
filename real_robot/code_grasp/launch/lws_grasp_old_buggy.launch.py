import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    package_name = 'mecharm_grasp_lws'

    share = get_package_share_directory(package_name)

    # ============================================================
    # 1. 获取原有 mycobot_description 的资源目录
    # ============================================================

    # Gazebo 的 model:// 资源根目录必须指向 share 的上一层，
    # 这样：
    # model://mycobot_description/urdf/xxx.dae
    # 才能解析到：
    # .../share/mycobot_description/urdf/xxx.dae
    mycobot_share = os.path.dirname(
        get_package_share_directory('mycobot_description')
    )

    # ============================================================
    # 2. Gazebo 模型资源路径
    #
    # URDF 中使用：
    # model://mycobot_description/urdf/mecharm_270_pi/xxx.dae
    #
    # 因此必须让 Gazebo 能找到 mycobot_description
    # ============================================================

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            mycobot_share,
            ':',
            os.path.join(share, 'urdf'),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]
    )

    # ============================================================
    # 3. 文件路径
    # ============================================================

    world_file = os.path.join(
        share,
        'worlds',
        'lws_grasp_world.sdf'
    )

    robot_xacro = os.path.join(
        share,
        'urdf',
        'mecharm_270_gazebo.urdf.xacro'
    )

    controllers_file = os.path.join(
        share,
        'config',
        'controllers.yaml'
    )

    params_file = os.path.join(
        share,
        'config',
        'lws_grasp_params.yaml'
    )

    # ============================================================
    # 4. Launch 参数
    # ============================================================

    gui = LaunchConfiguration('gui')
    task = LaunchConfiguration('task')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    task_arg = DeclareLaunchArgument(
        'task',
        default_value='true',
        description='Start grasp task'
    )

    # ============================================================
    # 5. Gazebo
    # ============================================================

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            world_file
        ],
        output='screen',
    )

    # ============================================================
    # 6. /clock bridge
    # ============================================================

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # ============================================================
    # 7. target_object odometry bridge
    # ============================================================

    object_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/target_object/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    # ============================================================
    # 8. target_object cmd_vel bridge
    # ============================================================

    object_cmd_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/target_object/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )

    # ============================================================
    # 9. robot_description
    # ============================================================

    robot_description = {
        'robot_description': os.popen(
            'xacro "{}" controllers_file:="{}"'.format(
                robot_xacro,
                controllers_file
            )
        ).read()
    }

    # ============================================================
    # 10. robot_state_publisher
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # ============================================================
    # 11. Spawn mechArm
    # ============================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'mecharm',
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '0.75'
        ],
        output='screen'
    )

    # ============================================================
    # 12. Joint State Broadcaster
    # ============================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # ============================================================
    # 13. Arm Controller
    # ============================================================

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # ============================================================
    # 14. Hand Controller
    # ============================================================

    hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # ============================================================
    # 15. LWS Grasp Task
    # ============================================================

    grasp_task = Node(
        package='mecharm_grasp_lws',
        executable='grasp_task_lws',
        parameters=[
            params_file
        ],
        output='screen',
        condition=IfCondition(task)
    )

    # ============================================================
    # 16. 控制器启动顺序
    #
    # spawn robot
    #      ↓
    # joint_state_broadcaster
    #      ↓
    # arm_controller + hand_controller
    #      ↓
    # grasp_task
    # ============================================================

    spawn_to_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster
            ]
        )
    )

    jsb_to_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                arm_controller,
                hand_controller
            ]
        )
    )

    hand_to_task = RegisterEventHandler(
        OnProcessExit(
            target_action=hand_controller,
            on_exit=[
                grasp_task
            ]
        )
    )

    # ============================================================
    # 17. LaunchDescription
    # ============================================================

    return LaunchDescription([

        gui_arg,
        task_arg,

        gazebo_resource_path,

        gazebo,

        clock_bridge,
        object_odom_bridge,
        object_cmd_bridge,

        robot_state_publisher,

        spawn_robot,

        spawn_to_jsb,
        jsb_to_controllers,
        hand_to_task,

    ])
