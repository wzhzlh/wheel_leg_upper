"""双并腿轮足机器人 MuJoCo 仿真启动文件。

链路：
    robot_state_publisher      URDF -> /robot_description + TF（腿/轮连杆）
    mujoco_ros2_control        加载 model 包的 scene.xml，提供 ros2_control 硬件接口
    controller_manager spawner 依次拉起 joint_state_broadcaster /
                               leg_position_controller / wheel_effort_controller
    rviz2（可选）              看 URDF 连杆姿态

用法：
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_rviz:=true
    ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false   # 无窗口（服务器上跑）

手动测试：
    # 腿关节摆一个屈膝站姿（顺序：左髋、左膝、右髋、右膝）
    ros2 topic pub --once /leg_position_controller/commands \\
        std_msgs/msg/Float64MultiArray "{data: [0.0, -0.4, 0.0, -0.4]}"
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

    urdf_path = os.path.join(model_share, "model", "wheel_leg.urdf")
    controller_yaml = os.path.join(model_share, "config", "wheel_leg_controllers.yaml")
    default_mjcf = os.path.join(model_share, "model", "scene.xml")

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
            "leg_position_controller",
            "wheel_effort_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
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
        robot_state_pub,
        mujoco,
        load_controllers,
        rviz2,
    ])
