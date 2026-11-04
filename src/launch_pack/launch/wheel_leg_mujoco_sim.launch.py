"""双并腿轮足机器人 MuJoCo 仿真启动文件。

链路：
    robot_state_publisher      URDF -> /robot_description + TF（腿/轮连杆）
    mujoco_ros2_control        加载 model 包的 scene.xml，提供 ros2_control 硬件接口
    controller_manager spawner 依次拉起 joint_state_broadcaster /
                               leg_effort_controller / wheel_effort_controller
    leg_calc/command_param     rqt 参数服务 -> /robot_command（50 Hz，上层意图）
    leg_calc/leg_control       实机 500 Hz 控制链：状态机 + Pitch LQR + VMC + 轮子分配
                               /robot_command + /imu/data + /joint_states
                               -> /joint_target（4 髋力矩）+ /wheel_target（轮速）
    leg_calc/sim_bridge        仿真 I/O 桥（use_control 控制）
                               /imu_site_imu_sensor/imu + /imu_site_pose_sensor/pose
                               -> /imu/data
                               /joint_target + /wheel_target
                               -> /leg_effort_controller/commands,
                                  /wheel_effort_controller/commands（轮速用 PI 换成力矩）
    rviz2（可选）              看 URDF 连杆姿态

用法：
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_rviz:=true
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false   # 无窗口（服务器上跑）

手动测试：
    # 四髋力矩为零（effort 命令，顺序：左前、左后、右前、右后）
    ros2 topic pub --once /leg_effort_controller/commands \\
        std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
    # 给两个轮子一个前向力矩
    ros2 topic pub -r 100 /wheel_effort_controller/commands \\
        std_msgs/msg/Float64MultiArray "{data: [0.2, 0.2]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_share = get_package_share_directory("model")
    leg_calc_share = get_package_share_directory("leg_calc")

    urdf_path = os.path.join(model_share, "model", "wheel_leg.urdf")
    controller_yaml = os.path.join(model_share, "config", "wheel_leg_controllers.yaml")
    default_mjcf = os.path.join(model_share, "model", "scene.xml")
    # leg_calc 参数：leg_control 段（控制环）
    leg_params = os.path.join(leg_calc_share, "config", "leg_control_params.yaml")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    # ---------------- launch 参数 ----------------
    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf,
        description="MuJoCo 使用的 MJCF/scene XML 路径",
    )
    show_gui_arg = DeclareLaunchArgument(
        "show_gui",
        default_value="true",
        description="是否打开 MuJoCo 可视化窗口",
    )
    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="是否同时启动 RViz2（需要图形界面）",
    )
    sim_freq_arg = DeclareLaunchArgument(
        "simulation_frequency",
        default_value="500.0",
        description="MuJoCo 物理步进频率 [Hz]",
    )
    rtf_arg = DeclareLaunchArgument(
        "real_time_factor",
        default_value="1.0",
        description="仿真实时倍率（>1 加速，<1 减速）",
    )
    use_command_server_arg = DeclareLaunchArgument(
        "use_command_server",
        default_value="true",
        description="是否启动 rqt 可调的上层指令参数节点（发布 /robot_command）",
    )
    use_control_arg = DeclareLaunchArgument(
        "use_control",
        default_value="true",
        description="是否启动 leg_control（实机控制算法）+ sim_bridge（仿真 I/O 桥）",
    )
    start_gate_arg = DeclareLaunchArgument(
        "start_gate",
        default_value="true",
        description="启动门控：控制器全部 active 前挂住机器人（不积分物理），避免启动阶段自由落体",
    )
    start_gate_hold_arg = DeclareLaunchArgument(
        "start_gate_hold",
        default_value="1.0",
        description="控制器全部 active 后继续挂架保持的时间 [s]，留给 leg_control 走完自检",
    )
    enable_arg = DeclareLaunchArgument(
        "enable",
        default_value="true",
        description="启动时是否直接使能平衡。仿真默认 true（否则一直停在 IDLE 不会站起来）；"
                    "想先手动检查可用 enable:=false，再 ros2 param set /command_param enable true",
    )

    # ---------------- 节点 ----------------
    # URDF 只负责 TF 与关节名，动力学由 MJCF 决定
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            {"robot_description": robot_desc},
            controller_yaml,
            {"simulation_frequency": ParameterValue(
                LaunchConfiguration("simulation_frequency"), value_type=float)},
            {"real_time_factor": ParameterValue(
                LaunchConfiguration("real_time_factor"), value_type=float)},
            {"robot_model_path": LaunchConfiguration("mjcf_path")},
            {"show_gui": ParameterValue(LaunchConfiguration("show_gui"), value_type=bool)},
            # 启动门控：joint_state_broadcaster / leg_effort_controller /
            # wheel_effort_controller 全部 active 之前不做物理积分
            {"start_gate_enabled": ParameterValue(
                LaunchConfiguration("start_gate"), value_type=bool)},
            {"start_gate_hold": ParameterValue(
                LaunchConfiguration("start_gate_hold"), value_type=float)},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # 控制器要等 hardware 起来后再 spawn，否则 controller_manager 里没有接口
    spawners = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "leg_effort_controller",
            "wheel_effort_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # ROS 2 原生参数服务：rqt_reconfigure → command_param → /robot_command
    command_param = Node(
        package="leg_calc",
        executable="command_param_node",
        name="command_param",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_command_server")),
        parameters=[leg_params,
                    {"enable": ParameterValue(LaunchConfiguration("enable"), value_type=bool)}],
    )

    # 复用实机 500 Hz 控制链：/robot_command + /imu/data + /joint_states
    #                          → /joint_target(力矩) + /wheel_target(轮速)
    # 仿真里把 startup_cycles 从 250 降到 100：0.2 s 就进 RUN，
    # 否则自检还没走完机器人已经倒了（无控制时 0.5 s 左右就超过 max_pitch）
    leg_control = Node(
        package="leg_calc",
        executable="leg_control_node",
        name="leg_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_control")),
        # 行驶由 LQR 的速度参考负责（仿真桥把 /wheel_target 当力矩用，力矩通道只有实机走）
        # max_yaw_rate 降到 0.10：原地偏航靠横向擦地，实测 0.10 rad/s 能稳定跟踪
        # （上层给 0.20 时实际只到 ~0.1，再往上就脱粘擦滑、把俯仰顶翻）。
        # 限幅后超范围的指令只会饱和，不会倒地
        # 仿真走力矩通道：leg_control 把 LQR u / 速度 P / 偏航力矩直接发到 /wheel_torque，
        # sim_bridge 以 N·m 下发（速度通道的 20·u rad/s 在 torque 模式会被当成 20 倍力矩）。
        # k_vel_p 必须给 0：速度由 LQR 的 k_x_dot 反馈负责，再叠一层会把速度极点推成不稳定
        # 仿真机的控制线程没有 CAP_SYS_NICE：SCHED_FIFO/nice 必然失败，只会刷一条 WARN；
        # IDE/浏览器抢时间片时控制环最长会被拖 60+ ms，0.10 s 的传感器超时会误判成
        # kFaultImu/kFaultJointState 而停机（实测组合测试里出现过一次 0x01）。
        # 仿真里放宽到 0.25 s（实机仍用 yaml 的 0.10 s，下位机有独立 RT 线程）。
        parameters=[leg_params, {
            "startup_cycles": 100,
            "max_yaw_rate": 0.10,
            "wheel_torque_enable": True,
            "k_vel_p": 0.0,
            # 力矩通道下已验证的线速度上限 0.10 m/s（再高会超调/擦滑），
            # 超范围指令只会饱和，不会把机器人冲出去
            "max_linear_velocity": 0.10,
            "imu_timeout": 0.25,
            "joint_state_timeout": 0.25,
            "wheel_state_timeout": 0.25,
            "rt_priority": 0,
            "rt_nice": 0,
        }],
    )

    # 仿真 I/O 桥：补 IMU 姿态，并把 joint_target / wheel_target 翻译成
    # leg_effort_controller / wheel_effort_controller 的命令
    sim_bridge = Node(
        package="leg_calc",
        executable="sim_bridge_node",
        name="sim_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_control")),
        parameters=[controller_yaml],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        output="screen",
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo 已启动，开始 spawn 控制器"),
                spawners,
            ],
        )
    )

    return LaunchDescription([
        mjcf_path_arg,
        show_gui_arg,
        show_rviz_arg,
        sim_freq_arg,
        rtf_arg,
        use_command_server_arg,
        use_control_arg,
        start_gate_arg,
        start_gate_hold_arg,
        enable_arg,
        robot_state_pub,
        mujoco,
        load_controllers,
        command_param,
        leg_control,
        sim_bridge,
        rviz2,
    ])
