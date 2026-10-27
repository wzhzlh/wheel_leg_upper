"""双并腿轮足机器人总启动文件。

启动链路：
    imu_ros2_device/ybimu_driver      /imu/data
    leg_calc/lqr_controller_node       /imu/data + /odom + /joint_states -> leg_calc/control
    leg_calc/vmc_controller_node       /joint_states -> 足端弹簧阻尼 + J^T -> joint_target
                                       （仅 use_vmc:=true 时启动）
    leg_calc/diff_drive_node           /cmd_vel -> 左右轮目标转速 -> wheel_target
                                       （仅 use_diff_drive:=true 时启动）
    usb_drive/usb_drive_node           leg_calc/control + wheel_target + joint_target
                                       -> USB-CDC -> 下位机
                                       下位机状态 -> /odom + /joint_states

用法：
    ros2 launch launch_pack wheel_leg_bringup.launch.py
    ros2 launch launch_pack wheel_leg_bringup.launch.py enable_motor:=false   # 只跑感知与 LQR
    ros2 launch launch_pack wheel_leg_bringup.launch.py use_vmc:=true         # 同时开足端 VMC
    ros2 launch launch_pack wheel_leg_bringup.launch.py use_diff_drive:=true  # 同时开差速行驶
    ros2 launch launch_pack wheel_leg_bringup.launch.py port:=/dev/ttyUSB0 \
        params_file:=/path/to/lqr_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------- 开关 ----------------
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='是否启动 YbImu 驱动')
    use_lqr_arg = DeclareLaunchArgument(
        'use_lqr', default_value='true',
        description='是否启动 LQR 控制器')
    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor', default_value='true',
        description='是否启动 usb_drive（会真实向电机下发指令，调试时置 false）')

    # ---------------- IMU ----------------
    port_arg = DeclareLaunchArgument(
        'port', default_value='', description='IMU 串口，留空则自动探测')
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame_id', default_value='imu_link', description='IMU 坐标系名')
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate', default_value='100.0', description='IMU 发布频率 [Hz]')

    # ---------------- LQR ----------------
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('leg_calc'), 'config', 'lqr_params.yaml']),
        description='LQR 控制器参数文件（模型参数 / Q / R / K(h) 系数 / 腿部几何）')

    # ---------------- 足端 VMC ----------------
    use_vmc_arg = DeclareLaunchArgument(
        'use_vmc', default_value='false',
        description='是否启动足端虚拟弹簧阻尼控制器（发布 joint_target 关节力矩）')
    vmc_params_arg = DeclareLaunchArgument(
        'vmc_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('leg_calc'), 'config', 'vmc_params.yaml']),
        description='VMC 参数文件（五杆几何 / Kp / Kd / 足端目标）')

    # ---------------- 差速底盘 ----------------
    use_diff_drive_arg = DeclareLaunchArgument(
        'use_diff_drive', default_value='false',
        description='是否启动差速底盘控制器（/cmd_vel → /wheel_target 左右轮目标转速）')
    diff_drive_params_arg = DeclareLaunchArgument(
        'diff_drive_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('leg_calc'), 'config', 'diff_drive_params.yaml']),
        description='差速底盘参数文件（轮距 / 轮半径 / 限幅 / 超时）')

    # ---------------- 下位机通信 ----------------
    vid_arg = DeclareLaunchArgument(
        'vid', default_value='1155', description='USB VID（0x0483 = 1155）')
    pid_arg = DeclareLaunchArgument(
        'pid', default_value='22336', description='USB PID（0x5740 = 22336）')
    send_rate_arg = DeclareLaunchArgument(
        'send_rate', default_value='100.0', description='下位机指令发送频率 [Hz]')
    wheel_mode_arg = DeclareLaunchArgument(
        'wheel_cmd_mode', default_value='velocity',
        description='轮子指令模式：velocity（u 折算轮速并积分成角度）/ position（u 当目标角）')

    # ---------------- 节点 ----------------
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

    lqr_controller = Node(
        package='leg_calc',
        executable='lqr_controller_node',
        name='leg_calc_lqr_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lqr')),
        parameters=[LaunchConfiguration('params_file')],
    )

    vmc_controller = Node(
        package='leg_calc',
        executable='vmc_controller_node',
        name='leg_calc_vmc_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_vmc')),
        parameters=[LaunchConfiguration('vmc_params_file')],
    )

    diff_drive = Node(
        package='leg_calc',
        executable='diff_drive_node',
        name='leg_calc_diff_drive',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_diff_drive')),
        parameters=[LaunchConfiguration('diff_drive_params_file')],
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
            'wheel_cmd_mode': LaunchConfiguration('wheel_cmd_mode'),
        }],
    )

    return LaunchDescription([
        use_imu_arg,
        use_lqr_arg,
        enable_motor_arg,
        port_arg,
        imu_frame_arg,
        imu_rate_arg,
        params_arg,
        use_vmc_arg,
        vmc_params_arg,
        use_diff_drive_arg,
        diff_drive_params_arg,
        vid_arg,
        pid_arg,
        send_rate_arg,
        wheel_mode_arg,
        imu_driver,
        lqr_controller,
        vmc_controller,
        diff_drive,
        usb_drive,
    ])
