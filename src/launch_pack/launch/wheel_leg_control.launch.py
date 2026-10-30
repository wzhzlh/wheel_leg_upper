"""分层控制启动文件（推荐入口）。

数据流：
    imu_ros2_device/ybimu_driver        /imu/data                    100 Hz
        │
    leg_calc/command_param_node          rqt 参数服务 → /robot_command   50 Hz
        │
    leg_calc/leg_control_node           /imu/data + /joint_states + /robot_command
        │                               → /joint_target (effort=τ)
        │                               → /wheel_target (ω_L, ω_R)    500 Hz
        │
    usb_drive/usb_drive_node            → USB-CDC → 下位机            200 Hz
                                        ← RobotState_t → /odom + /joint_states

层级职责：
    * 参数指令节点只更新“上层意图”，不参与控制；也可关闭后改用外部 /robot_command 源；
    * 控制节点在固定周期内完成 状态机 → Pitch LQR → 双侧 VMC → 轮子分配；
    * USB 节点只做搬运，状态帧超时立即改发安全帧。

用法：
    ros2 launch launch_pack wheel_leg_control.launch.py
    ros2 launch launch_pack wheel_leg_control.launch.py enable_motor:=false   # 干跑
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true', description='是否启动 YbImu 驱动')
    port_arg = DeclareLaunchArgument(
        'port', default_value='', description='IMU 串口，留空则自动探测')
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame_id', default_value='imu_link', description='IMU 坐标系名')
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate', default_value='200.0', description='IMU 发布频率 [Hz]')

    control_params_arg = DeclareLaunchArgument(
        'control_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('leg_calc'), 'config', 'leg_control_params.yaml']),
        description='控制节点和上层指令参数节点的参数文件')

    use_command_server_arg = DeclareLaunchArgument(
        'use_command_server', default_value='true',
        description='是否启动 rqt 可调的上层指令参数节点（发布 /robot_command）')

    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor', default_value='true',
        description='是否启动 usb_drive（会真实向电机下发指令）')
    vid_arg = DeclareLaunchArgument('vid', default_value='1155', description='USB VID')
    pid_arg = DeclareLaunchArgument('pid', default_value='22336', description='USB PID')
    send_rate_arg = DeclareLaunchArgument(
        'send_rate', default_value='200.0', description='下位机指令发送频率 [Hz]')

    imu_driver = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_imu')),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'publish_rate': ParameterValue(LaunchConfiguration('imu_rate'), value_type=float),
        }],
    )

    leg_control = Node(
        package='leg_calc',
        executable='leg_control_node',
        name='leg_control',
        output='screen',
        parameters=[LaunchConfiguration('control_params_file')],
    )

    # ROS 2 原生参数服务：rqt_reconfigure → command_param → /robot_command
    command_param = Node(
        package='leg_calc',
        executable='command_param_node',
        name='command_param',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_command_server')),
        parameters=[LaunchConfiguration('control_params_file')],
    )

    usb_drive = Node(
        package='usb_drive',
        executable='usb_drive_node',
        name='usb_drive_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_motor')),
        parameters=[{
            'vid': ParameterValue(LaunchConfiguration('vid'), value_type=int),
            'pid': ParameterValue(LaunchConfiguration('pid'), value_type=int),
            'send_rate': ParameterValue(LaunchConfiguration('send_rate'), value_type=float),
        }],
    )

    return LaunchDescription([
        use_imu_arg,
        port_arg,
        imu_frame_arg,
        imu_rate_arg,
        control_params_arg,
        use_command_server_arg,
        enable_motor_arg,
        vid_arg,
        pid_arg,
        send_rate_arg,
        imu_driver,
        leg_control,
        command_param,
        usb_drive,
    ])
