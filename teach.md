# 双并腿轮足机器人（wheel_leg）工程说明

本工程实现一条完整的自平衡控制链路：**IMU 姿态 → 欧拉角 → LQR 状态反馈 → USB-CDC → 下位机电机**，
配套拉格朗日动力学建模、小角度线性化、LQR 求解与 **增益随质心高度调度 K(h)**；
另有一套 **五杆腿的足端 VMC**（虚拟弹簧-阻尼 + 雅可比力矩映射，第 9 章）和一套
**差速底盘运动学**（`/robot_command` 速度目标 → 左右轮目标转速，第 10 章），
以及一套 **MuJoCo 仿真模型**（STL 网格的 URDF + 双五杆闭链 MJCF），可以完全不接硬件就把整机跑起来。

当前整机只有**一条实机主链路**：
`50 Hz 参数服务命令（rqt / ros2 param） → 500 Hz 统一控制环（状态机 + Pitch LQR + 双侧 VMC + 轮子分配）→ 200 Hz USB-CDC → 下位机`。
原来按话题拼装的独立 LQR / VMC / 差速节点已移除，算法保留在库和测试中；离线模型演示、增益扫描和独立差速节点不再作为可执行程序构建。

> 阅读顺序建议：第 1~2 章看全局 → 第 3 章查文件 → 第 5 章跑起来 → 第 6 章调参 → 第 9/10 章看 VMC 与差速 → 第 11 章看分层控制与状态机。
> 只想看仿真：直接跳到 **5.6 MuJoCo 仿真**（**先按那里的说明隔离 `ROS_DOMAIN_ID`**）；
> 想知道“怎么一步步调成能稳稳站住”的完整过程与最终参数：**12.11**；附录 C 是 URDF/MJCF 的约定表。

---

## 1. 系统架构与数据流

```
  ┌───────────────────────┐   50 Hz   ┌──────────────────────────────────────┐
  │ command_param_node    │──────────►│ /robot_command                       │
  │ 只改“上层意图”，不算控制 │  Float64  │ [v, ω, 质心高度, pitch, enable, estop] │
  └───────────────────────┘           └──────────────┬───────────────────────┘
                                                     │
  /imu/data ────┐
                ├──► 周期边界取一份一致快照 ──►┌──────────────────────────────────┐
  /joint_states ┘                              │ leg_control_node        500 Hz   │
                                               │ ① 状态机 / 安全监督              │
                                               │ ② Pitch LQR（全局，轮轴力矩 u）   │
                                               │ ③ 左腿 VMC + 右腿 VMC（关节力矩） │
                                               │ ④ 轮子控制 + 分配层（速度域）     │
                                               │ ⑤ 2D 里程计（轮弧长 + IMU yaw）→ /odom + TF │
                                               └───────┬───────────────┬──────────┘
                                          /joint_target│      /wheel_target│
                                           (effort=τ)  │     (ω_L, ω_R)   │
                                                       ▼                  ▼
                                               ┌──────────────────────────────────┐
                                               │ usb_drive_node          200 Hz   │
                                               │ 搬运 + 链路看门狗 + 安全帧        │
                                               └───────────────┬──────────────────┘
                                                               │ USB-CDC
                                                               ▼
                                               ┌──────────────────────────────────┐
                                               │ STM32：333 Hz 电机环 / CAN 执行   │
                                               └──────────────────────────────────┘
```

三个频率互不等待：参数服务 50 Hz 只更新意图，USB 200 Hz 只搬运最近一帧，
500 Hz 控制环只读最新快照；命令超时由状态机处理（降级 → 停机），不会被某个节点“卡住”。

| 环节 | 包 / 节点 | 关键话题 |
|---|---|---|
| 上层命令（50 Hz） | `leg_calc/command_param_node` | 发布 `/robot_command` |
| 统一控制（500 Hz） | `leg_calc/leg_control_node` | 订阅 `/imu/data` `/joint_states` `/robot_command`，发布 `/joint_target` `/wheel_target` `/odom` |
| 通信（200 Hz） | `usb_drive` | 订阅 `/joint_target` `/wheel_target`，发布 `/joint_states` `/usb_drive/state` |
| IMU 采集 | `imu_ros2_device` | 发布 `/imu/data` |
| 通信协议 | `usb_drive` | 96 B 目标帧 / 208 B 状态帧 |
| 物理仿真 | `mujoco_ros2_control` | `/joint_states`、`/leg_effort_controller/commands`、`/wheel_effort_controller/commands` |

LQR 的推导、线性化与增益调度见第 6 章；足端 VMC 见第 9 章；分层控制与状态机见第 11 章。

---

## 2. 目录结构

```
~/wheel_leg/
├── src/
│   ├── imu_ros2_device/         ← YbImu 驱动（ament_python, Python）
│   │   ├── imu_ros2_device/ybimu_driver.py  ← 串口→姿态融合→/imu/data
│   │   ├── setup.py / package.xml
│   │   └── resource/imu_ros2_device
│   ├── usb_drive/               ← USB-CDC 通信（ament_cmake, C++）
│   │   ├── include/usb_drive/
│   │   │   ├── data_pack.h          ← 上下位机二进制协议（编译期校验）
│   │   │   └── cdc_trans.hpp        ← CdcTrans(libusb 封装) + FrameAssembler
│   │   ├── src/
│   │   │   ├── cdc_trans.cpp
│   │   │   └── usb_drive_node.cpp   ← ROS 节点（收发 + 关节状态发布）
│   │   ├── test/test_frame_assembler.cpp
│   │   └── CMakeLists.txt
│   ├── launch_pack/             ← 唯一的启动包（实机 + 仿真）
│   │   ├── launch/wheel_leg_control.launch.py      ← 整机唯一入口
│   │   ├── launch/wheel_leg_mujoco_sim.launch.py   ← MuJoCo 仿真启动
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── leg_calc/                ← 算法库 + 控制节点（ament_cmake, C++）
│   │   ├── include/leg_calc/
│   │   │   ├── wheel_leg_model.hpp    ← 拉格朗日模型 + 解析线性化
│   │   │   ├── lqr.hpp                ← Riccati 求解 + K(h) 调度 + 腿部几何
│   │   │   ├── foot_vmc.hpp           ← 五杆正/逆运动学 + 雅可比 + 足端弹簧阻尼
│   │   │   ├── control_state.hpp      ← 状态机 + 安全监督（纯逻辑，可单测）
│   │   │   ├── robot_command.hpp      ← /robot_command 命令帧字段定义
│   │   │   ├── leg_control_types.hpp  ← ControllerConfig / StateSnapshot / ControlOutput
│   │   │   ├── leg_control_core.hpp   ← LegController：LQR + 双 VMC + 分配 + 状态机
│   │   │   ├── leg_control_io.hpp     ← ROS 消息 ↔ 快照 / 输出打包 / 轮端里程计
│   │   │   ├── quat_utils.hpp         ← 四元数 → 欧拉角
│   │   │   └── param_utils.hpp        ← 节点参数声明/读取
│   │   ├── src/
│   │   │   ├── wheel_leg_model.cpp
│   │   │   ├── lqr.cpp
│   │   │   ├── foot_vmc.cpp
│   │   │   ├── leg_control_node.cpp    ← 500 Hz 节点壳（参数 + 话题 + 定时器）
│   │   │   ├── leg_control_core.cpp    ← 控制计算（无 ROS 依赖）
│   │   │   ├── leg_control_io.cpp      ← 输入解析 + 输出打包（无控制逻辑）
│   │   │   ├── command_param_node.cpp ← 50 Hz 参数服务 → /robot_command
│   │   │   └── sim_bridge_node.cpp     ← 仿真 I/O 桥（仅仿真）
│   │   ├── config/
│   │   │   └── leg_control_params.yaml ← 控制环 + 参数服务全部参数（唯一入口）
│   │   └── test/
│   │       ├── test_wheel_leg_model.cpp
│   │       ├── test_foot_vmc.cpp
│   │       ├── test_control_state.cpp  ← 状态机 / 安全门限
│   │       └── test_leg_control_core.cpp← 核心：VMC 力矩 / 轮子输出 / 超时
│   ├── model/                   ← 仿真模型（ament_cmake）
│   │   ├── model/wheel_leg.urdf     ← 浮动基座 URDF（无 STL，几何体）
│   │   ├── model/wheel_leg.xml      ← MJCF：由 URDF 转写，含执行器/传感器
│   │   ├── model/scene.xml          ← 地面 + 灯光 + 加载 wheel_leg.xml
│   │   ├── config/wheel_leg_controllers.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── mujoco_ros2_control/     ← MuJoCo + ros2_control integration（ament_cmake）
│       ├── include/mujoco_ros2_control/ ...
│       ├── src/ ...
│       ├── scripts/{urdf2mjcf,xacro2mjcf}.py
│       └── CMakeLists.txt / package.xml
├── .gitignore
└── teach.md                    ← 本文件
```

要点：

- **主链路只有一个启动入口**：`wheel_leg_control.launch.py`。旧的独立 LQR / VMC / 差速节点与
  `wheel_leg_bringup.launch.py` 已移除；所需算法仍在 `leg_calc` 的库和单元测试中。
- `launch_pack` 只负责“把节点按正确参数拉起来”，不写业务逻辑。
- `usb_drive` 是唯一的硬件出口：控制器只发话题，不直接碰 USB。
- `leg_calc` 内部分四层：**算法库（lqr / foot_vmc / wheel_leg_model / control_state）
  → 数据类型（types）→ 控制核心（core，含轮速分配与里程计换算）
  → 输入输出（io）→ 节点壳（node）**，控制核心不依赖 ROS，可以直接在单测里喂快照。

### 下位机固件（不在本仓库）

`~/qq_down/stm32/one/`：

| 文件 | 作用 |
|---|---|
| `task/task.h` | 上下位机协议结构体（`RobotState_t` / `RobotTarget_t`）、`JointCalib_t`、任务声明 |
| `task/task.c` | 4 个 FreeRTOS 任务（优先级全为 1）：`wheel_control_task`（位置-速度串级 PID → M2006 电流，`vTaskDelayUntil` 3 ms ≈ 333 Hz）、`joint_control_task`（RobStride 运控帧，3 ms）、`state_send_task`（状态帧，10 ms = 100 Hz）、`target_receive_task`（命令接收）；`HAL_CAN_RxFifo0MsgPendingCallback` 里解析 0x201/0x202 与 RobStride 扩展帧 |
| `Core/Src/usb_trans.c` | CDC 接收组包（收到短包即视为一帧结束，缓冲 512 B） |
| `Core/Inc/RobStride2.h` / `Core/Src/RobStride2.c` | RobStride 电机运动控制与反馈解析 |

> 注意：固件当前**没有命令超时 / 看门狗**：USB 断开后 `rob_target` 保持最后一帧，
> 轮子会继续按旧目标运行。上位机侧的 `usb_drive` 已用“状态帧超时 → 发安全帧”兜底（第 11 章），
> 但从安全设计上仍建议在固件里补一个 `target_age > 100 ms` 的失力动作。

---

## 3. 各文件具体作用

### 3.1 imu_ros2_device

| 文件 | 作用 |
|---|---|
| `imu_ros2_device/ybimu_driver.py` | 打开 YbImu 串口（`port` 参数优先，失败后按 `fallback_ports` 依次试，每 2 s 重试），以 `publish_rate` 发布 `/imu/data`（加速度、角速度、四元数姿态）。读串口异常只告警不退出。 |
| `setup.py` | 安装 `ybimu_driver` 可执行入口（启动统一由 `launch_pack` 提供）。 |

参数：`port`(默认空=自动)、`fallback_ports`(`/dev/myimu`,`/dev/ttyUSB0..2`)、`frame_id`(`imu_link`)、`topic`(`imu/data`)、`publish_rate`(100)、`orientation_stddev`(0.01)。


### 3.2 leg_calc

| 文件 | 作用 |
|---|---|
| `wheel_leg_model.hpp/.cpp` | 3 自由度拉格朗日模型：M(q)、C(q, q̇)、G(q)、T(q) → A、B。非线性 `stepRK4` 仅用于仿真验证。 |
| `lqr.hpp/.cpp` | `solveLQR`（Riccati 迭代，连续 LQR）、`GainSchedule::gainAt(h)`（K(h) 调度）、`LegGeometry::comHeight(q)`。 |
| `foot_vmc.hpp/.cpp` | **足端 VMC**：五杆闭链正/逆运动学 `T(q)`/`T⁻¹(x)`、雅可比 `J`、`J⁻ᵀJ`、足端虚拟弹簧阻尼 `F=Kp·Δp−Kd·ṗ`、`τ=JᵀF`、`computeVmcLeg` 一站式接口。 |
| `control_state.hpp` | **状态机 + 安全监督（第 11 章）**：`ControlState{kInit,kIdle,kRun,kDegraded,kFault}`、`FaultBits`、`SafetySupervisor::update(SafetyInputs) → SafetyStatus`；含姿态类故障“保平衡 + 自动恢复”与倒伏判据（11.3）。纯逻辑、不依赖 ROS。 |
| `robot_command.hpp` | `/robot_command` 的字段定义（`v, ω, 质心高度, pitch目标, enable, estop`），发布端与接收端共用同一份下标。 |
| `leg_control_types.hpp` | 控制层的数据契约：`ControllerConfig`（全部可调参数）、`StateSnapshot`（传感器 + 命令 + 健康度）、`ControlOutput`（力矩/轮速/状态）。 |
| `leg_control_core.hpp/.cpp` | **控制核心**：`LegController::step(snapshot,dt)`，内含 h 估计、LQR、双腿 VMC、轮子分配、状态机推进；不依赖 ROS。 |
| `leg_control_io.hpp/.cpp` | **输入输出**：`SnapshotIO`（IMU/关节/命令 → 快照，含轮端里程计换算）、`makeJointTarget` / `makeWheelTarget` / `makeStateDebug`。 |
| `leg_control_node.cpp` | **500 Hz 节点壳**：参数声明与热更新、话题连接、周期边界参数切换、实时性统计与日志。不含控制公式。 |
| `command_param_node.cpp` | **50 Hz 上层参数服务节点**：通过 ROS 2 原生参数服务暴露 `enable`、`teleop`、`estop`、`linear_velocity`、`yaw_rate`、`com_height`、`pitch_target`，供 `rqt_reconfigure` / `ros2 param set` 动态修改，并周期发布 `/robot_command`。 |
| `sim_bridge_node.cpp` | **仿真 I/O 桥**：把 MuJoCo 的 `imu_site` 位姿 + gyro 合成为 `/imu/data`（姿态先做 `q_base ⊗ R_x(+90°)` 换到 z-up 机体系，否则 ZYX 退化、`pitch` 恒为 0）；把 `/joint_target`、`/wheel_target` 转为 ros2_control effort 命令（`wheel_mode: torque` 时轮子目标就是力矩，再经 `balance_leak_rate` 一阶低通）；订阅 `/mujoco/start_gate_open`，`wheel_arm_on_gate:=true` 时在门控放开的同一刻才开始下发轮子力矩。实机链路不启动。 |
| `quat_utils.hpp` | `quaternionToRpy`，按 ZYX 顺序，pitch = asin(2(wy - zx))。 |
| `param_utils.hpp` | `declareModelParams` / `readModelParams` 等，统一声明和读取模型物理参数。 |
| `config/leg_control_params.yaml` | 控制环 + 参数服务节点全部参数（唯一入口，见 6.9 速查）。 |
| `test/test_*.cpp` | 模型、VMC、状态机、控制核心的单元测试。 |

### 3.3 usb_drive

| 文件 | 作用 |
|---|---|
| `cdc_trans.hpp/.cpp` | **传输层（收发 + 拆帧）**。`CdcTrans`：libusb 封装，按 VID/PID 打开、卸载 `cdc_acm`、抢占接口、异步 bulk 接收（循环重提交）、同步 bulk 发送、热插拔注册、掉线 500 ms 自动重连；`close()` 会先取消传输并等回调结束再释放，避免 use-after-free。`FrameAssembler`：定长帧重组，USB bulk 会粘包/拆包，按 208 B 累积切帧；`reset()` 可在回调内安全调用（失步重同步）。 |
| `data_pack.h` | 与固件逐字节一致的协议：`Wheel_t`(8B)、`JointTarget_t`(20B)、`JointState_t`(48B)、`RobotTarget_t`(**96B**)、`RobotState_t`(**208B**)。带 `static_assert` 卡死长度与字段偏移。 |
| `usb_drive_node.cpp` | 通信节点，**默认 200 Hz** 下发（只搬运，不做控制计算）：下行把 `/wheel_target`（轮速）积分成角度目标 + 速度目标、把 `/joint_target` 打包成 4 关节运控目标；上行解析状态帧，发布 `joint_states`(2 轮 + 4 关节)、`usb_drive/state`(原始量)。**不发布 odom**（里程计由控制层换算）。三重兜底：① 指令超时 → 轮子位置保持、关节失力；② **状态帧超时（`state_timeout`，默认 0.5 s）→ 立即改发安全帧**；③ 退出时下发一次安全帧。`left/right_invert` 在上下行**同时**生效。 |
| `test/test_frame_assembler.cpp` | 5 个单测：协议布局、粘包、拆包、残帧、回调内 reset。 |

### 3.4 launch_pack

| 文件 | 作用 |
|---|---|
| `launch/wheel_leg_control.launch.py` | **整机唯一入口**：`ybimu_driver` + `command_param_node` + `leg_control_node` + `usb_drive_node`（200 Hz）；参数 `use_imu` / `port` / `imu_frame_id` / `imu_rate` / `control_params_file` / `use_command_server` / `enable_motor` / `vid` / `pid` / `send_rate`。 |
| `launch/wheel_leg_mujoco_sim.launch.py` | 仿真链路：`robot_state_publisher` + `mujoco_ros2_control` + 2 个 controller spawner + RViz；`model_file` 默认 `model/model/scene.xml`，`controller_config` 默认 `model/config/wheel_leg_controllers.yaml`；可选 `show_gui` / `show_rviz` / `simulation_frequency` / `real_time_factor` / `use_command_server` / `use_control`。 |

### 3.5 model（仿真模型与配置）

全部是资源文件，没有编译产物，`CMakeLists.txt` 只做 `install(DIRECTORY model config)`。

| 文件 | 作用 |
|---|---|
| `model/wheel_leg.urdf` | **整机 URDF**（SolidWorks 导出后手工修正：坐标系换成 REP-103、补关节限位、补 `<ros2_control>`、`base_link` 质心改用网格包围盒中心）。一条腿 = **五杆闭链**：`lf/lb_joint1`（驱动曲柄，effort）+ `lf/lb_joint2`（被动铰）+ `lefoot_joint`（轮，effort），10 个 STL 网格。髋轴 y=±0.131（前）/±0.1255（后），轮轴 y=±0.1525（轮距 0.305）。详见附录 C。 |
| `model/wheel_leg.xml` | 与 URDF 一一对应的 **MJCF**（CAD 坐标系 x 前 / y 下 / z 左）。`base_link` 带 `<freejoint>`（浮动基座），6 个驱动关节全是 `<motor>`（髋 ±12 / 轮 ±3 N·m），被动膝由两条 `<connect>`（`site1/site2`）闭合；被动关节与轮都给了 `armature`（2e-4 / 2e-5，见 12.5）；`keyframe "home"` 是唯一权威初始位姿；`<sensor>` 里挂 `gyro`/`accelerometer` 与 `framepos`/`framequat`（`imu_site`）。 |
| `model/scene.xml` | MuJoCo 场景：地面（checker 材质）+ 光照 + `include wheel_leg.xml`。`mjcf_path` 指向本文件而不是 `wheel_leg.xml`，否则仿真里没有地面。 |
| `config/wheel_leg_controllers.yaml` | 三个控制器：`joint_state_broadcaster`、`leg_effort_controller`（4 腿）、`wheel_effort_controller`（2 轮），`update_rate: 500`。 |

### 3.6 mujoco_ros2_control（第三方）

| 文件 | 作用 |
|---|---|
| `src/mujoco_system.cpp` | `ros2_control` 的 SystemInterface 实现。按**关节名**把 URDF 的 `<ros2_control>` 关节映射到 MJCF 的 `<joint>`（`mj_name2id`）；读 `qpos/qvel/qfrc_actuator` 作状态，写 `ctrl` 作命令。 |
| `src/mujoco_ros2_control_plugin.cpp` | 加载 MJCF、创建 `mjModel/mjData`、跑物理循环、发布 `/clock`、把 MJCF 里的 `<sensor>` 自动注册成 ROS 话题。 |
| `CMakeLists.txt` | 用 `FetchContent` 拉 MuJoCo 3.2.7 源码一起编译（首次构建耗时，见 5.0）。 |

> 执行器识别规则（踩坑点）：插件遍历 MJCF 的 actuator，按 `gainprm/biasprm` 判断类型。
> `motor` → EFFORT 接口；`position`（只给 `kp`，**不能带 `kv`**）→ POSITION 接口；
> 都不匹配时退回插件内部 PD，把力矩写进 `qfrc_applied`。

---


## 4. 话题一览（命名 & 单位）

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 驱动 → 控制器 | 四元数 + 角速度，frame 由 `imu_frame_id` 决定 |
| `/joint_states` | `sensor_msgs/JointState` | 通信 → 控制器 | 2 轮 + 4 关节（名称见参数） |
| `/joint_target` | `sensor_msgs/JointState` | 控制器 → 通信 | 4 个腿关节的 `position/velocity/effort`；`effort` 由 VMC 写入（力矩） |
| `/wheel_target` | `Float64MultiArray` | 控制器 → 通信 | `[左轮, 右轮]` 目标角速度 [rad/s]，由分配层合成 |
| `/robot_command` | `Float64MultiArray` | 参数服务 → 控制器 | `[v, ω, 质心高度, pitch目标, enable, estop]`（50 Hz，格式见 `robot_command.hpp`） |
| `/odom` | `nav_msgs/Odometry` | 控制器 → 调试 | 2D 里程计：`x/y` 由轮弧长沿 IMU yaw 分解，朝向取 IMU yaw（减初始零位）；同时发布 `odom→base_link` 的 `/tf` |
| `/usb_drive/state` | `Float64MultiArray` | 调试 | 原始反馈（轮角/轮速 + 4 关节角/角速度） |
| `/leg_control/state` | `Float64MultiArray` | 调试 | `[state, faults, θ, θ̇, x, ẋ, u, h, h_cmd, τ0..τ3, ω_L, ω_R]` |
| `/leg_control/timing` | `Float64MultiArray` | 调试/实时性 | `[周期均值 ms, 周期最大 ms, 执行均值 ms, 执行最大 ms, 超期次数]`，1 Hz |

仿真侧另有：`/leg_effort_controller/commands`、`/wheel_effort_controller/commands`（都是 `Float64MultiArray`）、
`/imu_site_pose_sensor/pose`（`geometry_msgs/PoseStamped`，`imu_site` 的世界位姿，含四元数；**不要**用
`base_link` 的 `framepos/framequat`，那对 body 返回的是惯性系，静止时就会报 roll≈−4°、yaw≈90°）、
`/imu_site_imu_sensor/imu`、`/mujoco/start_gate_open`（`std_msgs/Bool`，latched；`sim_bridge` 据此决定
什么时候开始给轮子力矩，见 12.2）、`/tf`、`/tf_static`。

---

## 5. 如何试用

### 5.0 一次性编译

```bash
cd /home/zhangjiayi/wheel_leg
source /opt/ros/humble/setup.bash
colcon build            # 或 colcon build --packages-select leg_calc
source install/setup.bash
```

**首次编译有两件事要提前知道：**

1. `mujoco_ros2_control` 用 `FetchContent` 从 GitHub 拉 MuJoCo 3.2.7 源码一起编译，第一次会下载几百 MB、编译十几分钟。只要 `src` 里放了这个包，`colcon build` 就会带上它。
2. 拉依赖时如果卡在 `Failed to clone repository: 'https://github.com/gabime/spdlog.git'`（HTTP 408），是网络抖动，不是代码问题。解决办法是把仓库提前 clone 到本地，用 `FETCHCONTENT_SOURCE_DIR_*` 指过去：

```bash
mkdir -p ~/.cache/wheel_leg_deps
git clone --depth 1 --branch v1.9.2 https://github.com/gabime/spdlog.git ~/.cache/wheel_leg_deps/spdlog
git clone --depth 1 --branch v1.3.2 https://github.com/USCiLab/cereal.git  ~/.cache/wheel_leg_deps/cereal

colcon build --packages-select mujoco_ros2_control \
  --cmake-args \
  -DFETCHCONTENT_SOURCE_DIR_SPDLOG_LIB=$HOME/.cache/wheel_leg_deps/spdlog \
  -DFETCHCONTENT_SOURCE_DIR_CEREAL_LIB=$HOME/.cache/wheel_leg_deps/cereal
```

这两个 `-D` 会写进 `build/mujoco_ros2_control/CMakeCache.txt`，**之后直接 `colcon build` 即可**，只有删掉 `build/` 后才需要重新带上。
`spdlog`/`cereal` 的 commit 必须与 sdflib 要求的一致（`eb32206…` / `ebef1e9…`），上面对应的 tag 就是这两个。


### 5.1 第一层：离线模型验证

当前仓库不再构建 `model_demo_node` 和 `gain_table` 可执行文件。模型、LQR、VMC 与状态机
通过单元测试验证；现有 `k_coeffs` 直接配置在 YAML 中：

```bash
colcon test --packages-select leg_calc
colcon test-result --verbose --test-result-base build/leg_calc
```

### 5.2 第二层：控制环（不接电机）

```bash
# 只跑控制环，没有 IMU / 关节反馈时状态机会停在 INIT（这是预期行为）
ros2 launch launch_pack wheel_leg_control.launch.py enable_motor:=false use_imu:=false

ros2 topic echo /leg_control/state     # [state, faults, θ, θ̇, x, ẋ, u, h, h_cmd, τ0..3, ωL, ωR]
ros2 topic echo /leg_control/timing    # 周期/执行时间统计
```

### 5.3 第三层：整机（唯一入口）

```bash
# 终端 1：IMU 驱动 + 50 Hz 参数服务 + 500 Hz 控制环 + USB 通信
ros2 launch launch_pack wheel_leg_control.launch.py

# 终端 2：打开 rqt 参数界面
rqt
# Plugins → Configuration → Dynamic Reconfigure → 选择 /command_param
```

`command_param_node` 已由 launch 默认启动，参数修改后会在下一个 50 Hz 周期发布到 `/robot_command`。
模式对应关系如下：

| `enable` | `teleop` | `estop` | 控制状态 |
|---|---|---|---|
| `false` | 任意 | 任意 | ZERO（失能） |
| `true` | `false` | `false` | STAND（站立保持，速度强制为 0） |
| `true` | `true` | `false` | TELEOP（使用 `linear_velocity` / `yaw_rate`） |
| 任意 | 任意 | `true` | 急停 |

也可用命令行直接改：

```bash
ros2 param set /command_param enable true          # 站立
ros2 param set /command_param teleop true          # 进入遥控
ros2 param set /command_param linear_velocity 0.3  # 前进 0.3 m/s
ros2 param set /command_param yaw_rate 0.2         # 左转 0.2 rad/s

# 安全停车：先清零速度并退出遥控，再失能
ros2 param set /command_param linear_velocity 0.0
ros2 param set /command_param yaw_rate 0.0
ros2 param set /command_param teleop false
ros2 param set /command_param enable false
```

`estop=true` 会立即急停；FAULT 是锁存的，关闭 `estop` 后仍需先 `enable=false`，再重新使能。

先把机器吊起/架空，确认：

```bash
ros2 topic hz /joint_states        # 应约 100 Hz（下位机 10 ms 上报）
ros2 topic echo /usb_drive/state   # 前 4 个数是左右轮角/角速度，其后是 4 关节角/角速度
ros2 topic echo /leg_control/state # state=2 且 faults=0 才允许落地
```

**上电前务必确认 `theta_offset` 已标定**（见 6.2），否则一上电就会输出饱和力矩。

### 5.4 分段调试（同一个 launch，用开关裁链路）

```bash
# 只跑 IMU 驱动
ros2 launch launch_pack wheel_leg_control.launch.py enable_motor:=false port:=/dev/ttyUSB0

# 控制环 + 通信，没有 IMU（状态机停在 INIT）
ros2 launch launch_pack wheel_leg_control.launch.py use_imu:=false

# 只跑下位机通信（不带感知与控制）
ros2 launch launch_pack wheel_leg_control.launch.py use_imu:=false
```

### 5.5 记录与可视化

```bash
ros2 bag record /imu/data /joint_states /robot_command /joint_target /wheel_target                /leg_control/state /leg_control/timing
ros2 run plotjuggler plotjuggler
```

### 5.6 MuJoCo 仿真（不接任何硬件）

> **先隔离 ROS domain，否则一切都不准（2026-10-30 实测教训）**
>
> 这台机器上还跑着另一套机器人栈（`/AT_R2/ros_gz_bridge`、`/backend_node`、`/zlio` 等），
> 它在**同一个 domain 0** 上往 `/joint_states` 发另一种机器人（`base_to_chassis`、
> `front_left_steer_joint`…）、还共用 `/clock`。结果是 `leg_control` 的关节/轮反馈时有时无：
> 故障位在 `0x22 ↔ 0x00` 反复跳、自检永远攒不够 → 停在 `INIT`；数据混用后仿真会直接飞出去
> （见过 z=32 m、x=−124 m）。这类进程常常在容器里，`ps` 看不到也杀不掉，只能靠 domain 隔离：
>
> ```bash
> export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1   # 本次所有测试都在 42 上
> ```
>
> 每一行 `ros2` 命令（launch、`ros2 param set`、记录脚本）都要带这两个环境变量。

**启动**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1

ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py                      # 带 MuJoCo 窗口
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false      # 服务器/无显示器
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_rviz:=true      # 额外开 RViz（需手动加 RobotModel）
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py simulation_frequency:=1000.0 real_time_factor:=0.5
```

启动成功的标志（日志里依次出现）：

```
loaded mujoco model
[wheel_leg_system]: lf_joint1 / lb_joint1 / rf_joint1 / rb_joint1 /
                    lefoot_joint / rifoot_joint                  ← 6 个关节全部匹配
[lf_joint1_act]: added effort actuator for joint: lf_joint1      ← 6 个全是 effort（motor）
...  rifoot_joint_act 同理
启动门控已启用 → 控制器已全部 active：joint_state_broadcaster /
                 leg_effort_controller / wheel_effort_controller → 启动门控放开
```

**手动摆姿态 / 转轮子**

```bash
# 4 个髋（曲柄）力矩，顺序固定为 [左前, 左后, 右前, 右后]，单位 N·m
ros2 topic pub --once /leg_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# 轮子力矩（N·m），持续发才会持续给力矩
ros2 topic pub -r 100 /wheel_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2, 0.2]}"

# 看回读状态（下面两个是 MJCF 传感器原始话题，sim_bridge 会把它们换算成 /imu/data）
ros2 topic echo /joint_states
ros2 topic echo /imu_site_imu_sensor/imu
ros2 topic echo /base_link_pose_sensor/pose
```

**验证仿真是否"接对了"**

1. 给 `lf_joint1` 一个正力矩，`/joint_states` 里 `lf_joint1.position` 应增大；左右腿在世界系里是
   镜像的（右腿整体绕 x 转了 180°），同一个正力矩会让两侧轮轴往**相反方向**走，见 12.7；
2. 发正力矩 `[0.2, 0.2]`，两个轮子的 `position` 都应单调增加（MJCF 里左轮轴被翻成 +z，与右轮同向）；
3. `/clock` 频率应等于 `simulation_frequency`。

**几个必须知道的行为**

- 整机是**浮动基座**（MJCF 里 `base_link` 带 `freejoint`），不会自动站稳，腿一伸直就会倒下 —— 这是正常的，不是模型错了。想看它站着，需要闭环控制（见第 11 章）。
- 6 个关节**全是力矩直通**（MJCF `motor`）：4 髋 ±12 N·m、2 轮 ±3 N·m，没有 position actuator，
  所以腿的姿态必须靠 VMC 闭环维持。
- `u_max: 0.40`（轮地摩擦上限）远小于 MJCF 的 ±3，跑闭环时不会撞到 `ctrlrange`；`joint_torque_max: 8.0`
  同理低于髋的 ±12。
- `use_sim_time`：launch 没有统一打开。要让 `leg_calc` 吃仿真时间，需要给节点加 `-p use_sim_time:=true`，否则它用墙钟，和 `/clock` 会不同步。
- **轮子力矩什么时候开始下发**：`sim_bridge` 现在订阅门控的 latched 话题 `/mujoco/start_gate_open`，
  物理步进真正放开的那一刻才给轮子力矩（`wheel_arm_on_gate: true`）。以前用固定
  `wheel_start_delay=1.45 s` 计时，而门控放开时间在 1.12～1.60 s 之间浮动，会出现“挂架还夹着、轮子先转”，
  启动瞬态一度冲到 x=0.44 m；改成跟门控后启动瞬态只有 x∈[−0.007, 0.055]、θ∈[−0.017, 0.003]。


### 5.7 足端 VMC 检查（不需要电机）

VMC 已经并入控制环，不再需要单独节点。用假关节角驱动：

```bash
# 1) 只起控制环
ros2 launch launch_pack wheel_leg_control.launch.py enable_motor:=false use_imu:=false

# 2) 另开终端：IMU（pitch≈0）+ 关节角（home 姿态：左腿 0，右腿补偿零位不对称），50 Hz 以上持续发
ros2 topic pub -r 50 /joint_states sensor_msgs/msg/JointState   "{name: [lefoot_joint, rifoot_joint, lf_joint1, lb_joint1,
           rf_joint1, rb_joint1],
    position: [0,0,0,0,-0.2259,0.2301], velocity: [0,0,0,0,0,0]}"

# 3) 把目标质心高度给得比 home（0.0834）高 11.6 mm，腿应该"顶住"
ros2 topic pub -r 50 /robot_command std_msgs/msg/Float64MultiArray "{data: [0,0,0.095,0,1,0]}"

# 4) 看结果：foot_y 目标 = 0.095 + 0.0338 = 0.1288（实际 0.1172），τ 应非零且被 joint_torque_max=8 限幅
ros2 topic echo /leg_control/state
```

> 关节名只认 `leg_joint_names` / `wheel_joint_names` 里那 6 个，被动膝（`*_joint2`）不用发。

### 5.8 差速行驶检查

```bash
# 命令帧 [v, ω, 质心高度, pitch, enable, estop]；直行 v=0.5，左转 ω=1.0
ros2 topic pub -r 50 /robot_command std_msgs/msg/Float64MultiArray "{data: [0.5,0,0.0834,0,1,0]}"
ros2 topic pub -r 50 /robot_command std_msgs/msg/Float64MultiArray "{data: [0,1.0,0.0834,0,1,0]}"

ros2 topic echo /wheel_target    # 直行两轮同号，左转左轮 < 右轮
```

---

## 6. 调参手册


### 6.0 调参顺序（不要跳步）

```
⓪ 先确认环境干净：仿真必须隔离 ROS domain（5.6），否则反馈是别人的数据，调什么都是白费
   ↓
① 机械标定（关节零位/方向、theta_offset）
   ↓
② 模型参数（M, m, h, I, r, Jw，直接由 URDF 推导，见 6.1）+ 五杆几何 l1..l5 / com_offset
   ↓
③ 权重 Q/R → 观察台架响应（先 use_measured_height:=false 固定 h）
   ↓
④ 打开高度调度：校验 YAML 中的 k_coeffs → online 对比固定/调度增益
   ↓
⑤ 增益与限幅对齐：先保证 u 不长期贴 u_max（6.3/6.4），再谈上调 u_max
   ↓
⑥ 通信与限幅（send_rate, u_max, command_timeout）
   ↓
⑦ 整机试跑 → 按 12.10 的 7/8 项检查站立质量与扰动恢复
```

### 6.1 物理参数怎么给

**首选从 `src/model/model/wheel_leg.urdf` 算**（当前 yaml 里的值就是这么来的），称重/悬挂法只用来交叉核对。

| 参数 | 含义 | 从 URDF 怎么拿 | 当前值 |
|---|---|---|---|
| `cart_mass` M | 两轮质量之和 [kg] | `lefoot_link` + `rifoot_link` 的 `<mass>` | 0.285（0.130114 + 0.155050） |
| `pendulum_mass` m | 除两轮外 9 个连杆质量之和 [kg] | 1.667452(base) + 0.077208(4 曲柄) + 0.159329(4 连杆) | 1.904 |
| `com_length` h | **轮轴到摆杆质心**距离 [m] | home 姿态合成质心 z − 轮轴 z | 0.0834（0.115454 − 0.032089） |
| `com_inertia` I | 摆杆绕自身质心 pitch 惯量 [kg·m²] | 各连杆 izz 投影到 base 的 y 轴 + 平行轴项 `m·d²` | 0.00255 |
| `wheel_radius` r | 轮半径 [m] | 轮子 STL 的轮胎外缘（MJCF 接触圆柱用 0.032 是软接触近似） | 0.030 |
| `wheel_inertia` Jw | 两轮绕轮轴等效转动惯量 [kg·m²] | 两轮 `izz` 之和（轮轴平行连杆 z 轴，质心只沿轴向偏 34 mm，不改 izz） | 5.32e-5（2.339e-5 + 2.983e-5） |

> 等效平动质量 `M_eff = M + Jw/r²` 由代码内部计算，不用手填（当前 = 0.285 + 5.32e-5/0.03² = 0.344 kg）。

> **home 姿态 ≠ 关节全零**：URDF 的左右腿零位不对称，关节全零时左轮比右轮高 13.5 mm（`wheel_leg.xml`
> 的 `keyframe "home"` 注释里有实测值），要用 MJCF home 那组角算：左腿全零、
> `rf1=-0.2259 / rf2=+0.3339 / rb1=+0.2301 / rb2=-0.3302`。图省事用全零会把轮轴高度算成 0.0254 m，
> h 从 0.0834 变成 0.0895（偏大 7%）。

### 6.2 θ 的零位：`theta_offset`

1. 把机器摆到「你要当作直立平衡点」的姿态并静止；
2. 读 `/leg_control/state` 的第 3 个数（`θ`，单位 rad），记为 `θ0`；
3. 令 `theta_offset = θ0`（代码里 `θ = pitch − theta_offset`）。

运行时可直接改，不用重启：

```bash
ros2 param set /leg_control theta_offset 0.02
```

如果 IMU 装成侧向的（俯仰在 roll 上），改 `theta_axis: roll`。

### 6.3 权重 Q / R

代价函数 `J = ∫(xᵀQx + uᵀRu)dt`，`q_diag = [q_x, q_θ, q_ẋ, q_θ̇]`。

| 想要的效果 | 怎么改 |
|---|---|
| 摆杆更"硬"、倒得少 | 增大 `q_θ`（默认 600，可试 1000~5000） |
| 位置更"粘"、少漂 | 增大 `q_x`（默认 100） |
| 减少来回晃动、抑制速度 | 增大 `q_ẋ`、`q_θ̇`（默认 10 / 5） |
| 省力、动作更柔和 | 减小 `q_θ` 或增大 `r_weight` |
| 抖动/嗡鸣 | 增大 `r_weight`（=1 → 3~10），或降 `control_rate` |

经验：默认 `q_diag = [100, 600, 10, 5]`，即 `q_θ : q_x ≈ 6:1`；想更"站得死"就往 `[100, 2000, 10, 1]`
（50:1）那侧调。`r_weight` 从 1 开始，翻倍调。

改完注意闭环极点应该全部为负：

```bash
ros2 param set /leg_control q_diag "[20.0, 2000.0, 1.0, 1.0]"
# 参数回调会重新求解并在日志里打印名义高度处的 K
```

**关键：增益必须和 `u_max` 匹配，否则“站住”只是极限环（2026-10-30 实测）**

`q_diag=[50,500,10,1]` 解出的 `K≈[−7.07, −27.1, −6.9, −1.7]`，而 `u_max=0.4 N·m`：
俯仰到 **0.015 rad（0.9°）** 力矩就饱和。机器人不会倒，但会进入 **±0.3 m 的极限环**
（轮速冲到 ±18 rad/s、`u` 常年贴 ±0.4、饱和占比 **86%**）。把这组增益整体缩到 **1/6**
（`K≈[−1.18, −4.52, −1.16, −0.28]`）后饱和降到 **0%**，稳态 x∈[0.020,0.030]、θ≈−0.006、u≈±0.01。

原因是这台车稳住倒立摆只需要 `k_θ ≳ 0.66 N·m/rad`（见附录 A 的 `mgh` 与 `B`），而 LQR 给 27 相当于
“要求 0.9° 内就回到零”，执行器做不到，于是全程 bang-bang。**判据**：正常站立时
`/leg_control/state[6]`（即 `u`）应接近 0；长期贴边就把 K 整体缩小（或增大 `r_weight`）。

本仓库现在用 `gain_mode: quadratic` + `k_coeffs` 只填常数项，把这个 1/6 增益直接写死：

```yaml
gain_mode: quadratic
k_coeffs: [0.0,0.0,-1.18,  0.0,0.0,-4.52,  0.0,0.0,-1.16,  0.0,0.0,-0.28]
```

### 6.4 `u_max` 与饱和

`u_max`（默认 0.40 N·m）是平衡力矩硬限幅，取值来自轮地摩擦上限（`μ·N·r ≈ 0.4`，再大只会打滑、
让里程计失真），不是电机上限（MJCF/实机轮子都有 3 N·m 的余量）。若 `/leg_control/state` 里的 `u` 长期贴边，说明：
- θ 零位没标定（最常见）；
- 增益过大（Q 太大/R 太小）；
- 腿的 VMC 目标高度不合理（`h` 估偏 → K 偏 → 一直大力修）。

调试时可临时调小 `u_max`（如 0.1）再逐步放开；要真的验证轮子能出多大力，直接给 `/wheel_target`。

**先缩增益，再考虑抬 `u_max`**。2026-10-30 的极限环就是“增益过大”这一条（见 6.3）。
如果确实需要更大控制权限：场景地面 `friction=1.2`、总法向力约 21 N，理论上限 `μ·N·r ≈ 0.76 N·m`，
抬到 0.5～0.6 还能留 20%～35% 的摩擦余量；再高就开始打滑，`/odom` 和 LQR 的 `x/ẋ` 会先失真。

### 6.5 腿部几何与关节标定

上位机现在走**五杆**：由 4 个髋关节角经 `joint_sign/joint_offset` 换成 α/β → 五杆正运动学得足端
`foot_y`（髋轴到轮轴的竖直距离）→ `h = foot_y + com_offset`（左右腿取平均，再用 `h_min/h_max` 截断）。
`com_offset` 是「摆杆质心相对髋轴的竖直偏置」，质心在髋下方为**负**（当前 −0.0338 m）。

`thigh` / `shank` 只剩 `LegGeometry::comHeight()` 这个串联 2R 接口在用（单测和遗留调用），
不再参与 `h` 的计算，值保持 0.15/0.15 即可，改腿长不用管它们。

下位机（`stm32/one/task/task.c`）：`joint_calib[4] = {{pos_offset, motor_inv}, ...}`

```
关节角 q   = motor_inv * (θ_motor - pos_offset)
θ_motor    = motor_inv * q + pos_offset     （下发时反向换算）
```

标定步骤：

1. `joint_calib` 全填 `{0.0f, 1}` 烧录（默认即如此）；
2. `ros2 topic echo /usb_drive/state`，第 5~12 个数为 4 关节的 `rad, omega`（此时是电机原始角）；
3. 把腿摆到目标零位姿态，记下每个关节的 `rad` → 填入 `pos_offset`；
4. 手动正转关节，看 `rad` 增大还是减小 → 决定 `motor_inv = +1/−1`；
5. 重新烧录，确认 `/joint_states` 零位为 0、正向为正。

> 标定后，`com_offset` / `com_length` 必须按**这个零位姿态**（= home）测量/复算，见 6.1。

没标定时可先关闭高度调度：

```bash
ros2 param set /leg_control use_measured_height false
```

### 6.6 K(h) 二次拟合流程

> 当前仓库未构建 `gain_table` 工具，以下说明用于维护和核验已有 `k_coeffs`；
> 如需重新扫描，需要恢复离线拟合工具。

**第 1 步：生成数据并拟合**（工具输出示例）

```
h[m],k_x,k_theta,k_x_dot,k_theta_dot
0.1800,-3.162278,-27.795051,-4.205510,-2.498791
...
# 二次拟合 K_j(h) = c2*h^2 + c1*h + c0，最大残差 0.008316
    k_coeffs: [
      +0.000000, -0.000000, -3.162278,  # k_x
      -5.012369, -1.910371, -27.280468,  # k_theta
      -0.365440, -0.153980, -4.165307,  # k_x_dot
      -4.258899, -2.213014, -1.955853,  # k_theta_dot
    ]
```

> ⚠ 上面这段数字是**旧模型**（`h≈0.18 m`、`M=0.31/m=1.90`）拟合出来的，只作格式示例。
> 现在 `h` 的实际工作区间是 0.065~0.100（home 0.0834）、模型参数也换成了 URDF 值，
> 要切 `quadratic` 必须用当前 `q_diag/r_weight` 重新扫描拟合，否则 K 会明显偏。

**第 2 步：填进 `config/leg_control_params.yaml` 的 `k_coeffs`**（行序 = `[k_x, k_theta, k_x_dot, k_theta_dot]`，列序 = `[h², h, 1]`），并把 `gain_mode` 设为 `quadratic`。

**第 3 步：在线核对**（切换只看效果，不必重启）：

```bash
# 用拟合值
ros2 param set /leg_control k_coeffs "[0,-0,-3.162278, -5.0124,-1.9104,-27.2805, \
  -0.3654,-0.1540,-4.1653, -4.2589,-2.2130,-1.9559]"
# 切回现场解 LQR 对照（把 k_coeffs 全置 0 即可）
ros2 param set /leg_control k_coeffs "[0,0,0, 0,0,0, 0,0,0, 0,0,0]"
```

对比 `/leg_control/state` 里的 `h` 与按 6.5 公式算出的理论 `h`，偏差应 < 1%。

> 备注：`k_x = -√(q_x/r_weight)` 与 h 无关，拟合出来 c2=c1=0 属正常。


### 6.7 轮子方向与限幅

| 参数（`usb_drive`） | 默认 | 说明 |
|---|---|---|
| `left_invert` / `right_invert` | false | 左右轮镜像，上下行同时生效 |
| `max_wheel_omega` | 20.0 | 轮速限幅 [rad/s]（最后一道） |

控制层（`leg_control`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `balance_torque_to_omega` | 1.0 | LQR 力矩 → 轮速折算 [rad/s per N·m]，实机标定 |
| `max_balance_omega` | 8.0 | 平衡分量限幅 [rad/s] |
| `max_drive_omega` | 12.0 | 行驶分量限幅 [rad/s] |

改轮速前先确认 `odom` 里的 `x` 方向与实物一致（推车向前 `x` 增大），否则 `left/right_invert` 反了。

### 6.8 通信参数

| 参数（`usb_drive`） | 默认 | 说明 |
|---|---|---|
| `vid` / `pid` | 1155 / 22336 | 0x0483 / 0x5740，STM32 虚拟串口 |
| `interface` / `ep_in` / `ep_out` | 1 / 0x81 / 0x01 | CDC 数据接口与端点，一般不用改 |
| `send_rate` | 200 | 下行帧率（5 ms），低于固件控制环频率即可，不要低于 50 |
| `state_timeout` | 0.5 | 状态帧超时 [s]，超时立即改发安全帧 |
| `command_timeout` | 0.3 | 轮速/关节指令超时 [s]，超时后轮子位置保持、关节失力 |
| `joint_kp` / `joint_kd` | 0 / 0 | 关节运控 PD；0 表示纯力矩（VMC 模式） |

免 sudo 访问（可选）：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-wheel-leg.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 6.9 `leg_control_params.yaml` 参数速查

| 组 | 参数 | 默认 |
|---|---|---|
| 频率 | `control_rate` | 500 Hz |
| 模型 | `cart_mass` / `pendulum_mass` / `com_length` / `com_inertia` / `wheel_radius` / `wheel_inertia` | 0.285 / 1.904 / 0.0834 / 0.00255 / 0.030 / 5.32e-5（URDF 推导，见 6.1） |
| 权重 | `q_diag` / `r_weight` | [50, 500, 10, 1] / 1.0（`quadratic` 模式下不参与计算，只作参考） |
| 增益 | `gain_mode` / `k_coeffs` / `h_min` / `h_max` | quadratic / `[−1.18, −4.52, −1.16, −0.28]` 的常数项（LQR 的 1/6，见 6.3）/ 0.06 / 0.12；`online_lqr` 仍可用，k_coeffs 全零即退回 |
| 高度 | `com_height_default` / `_min` / `_max` / `com_offset` / `use_measured_height` | 0.0834 / 0.065 / 0.100 / -0.0338 m / true |
| 平衡 | `theta_axis` / `theta_offset` / `u_max` | pitch / 0.0 / 0.40 N·m |
| VMC | `l1..l5` / `kp_x,kp_y` / `kd_x,kd_y` / `force_max_*` / `joint_torque_max` / `joint_torque_sign` | 见第 9 章 |
| 关节角标定 | `joint_sign` / `joint_offset`（4 个数，α/β = sign·q + offset） | 见 12.7 |
| 高度斜坡 | `com_height_rate` | 0.30 m·s⁻¹ |
| 轮子 | `wheel_separation` / `max_wheel_omega` / `max_balance_omega` / `max_drive_omega` / `balance_torque_to_omega` / `balance_leak_rate` | 0.305 / 20 / 8 / 12 / 1.0 / 60 s⁻¹（力矩指令一阶低通，60 ≈ 8 ms） |
| 安全 | `imu_timeout` / `joint_state_timeout` / `command_timeout` / `command_fault_timeout` / `max_pitch` / `max_pitch_rate` / `startup_cycles` | 0.10 / 0.10 / 0.30 / 1.00 s / **0.35 rad（20°）** / **3.0** / 250 |
| 话题 | `imu_topic` / `joint_states_topic` / `command_topic` / `joint_target_topic` / `wheel_target_topic` / `odom_topic` | imu/data / joint_states / robot_command / joint_target / wheel_target / odom |
| 参数服务 | `enable` / `teleop` / `estop` / `linear_velocity` / `yaw_rate` / `com_height` / `pitch_target` | false / false / false / 0.0 / 0.0 / 0.0834 / 0.0 |
| 参数服务启动配置 | `command_topic` / `publish_rate` / `max_linear` / `max_yaw` / `com_height_min` / `com_height_max` / `max_pitch_target` | robot_command / 50 Hz / 1.0 / 3.0 / 0.065 / 0.100 / 0.20 |

`leg_control` 的控制参数支持 `ros2 param set /leg_control <名字> <值>`；上层指令参数则通过
`ros2 param set /command_param <名字> <值>` 或 rqt 修改，非法值会被拒绝并返回原因。

**物理参数只有一个权威来源**：`wheel_radius`、`wheel_separation` 只在这一个文件出现一次，
分配层与 `/odom` 都用它；`usb_drive` 不再读取任何物理量。

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `dlsym`/`libusb` 打开失败 | `cdc_acm` 占用或权限不足 | 节点会自动 `detach_kernel_driver`；仍失败就加 udev 规则 |
| 一直收到“非法状态帧” | 收到残缺/噪声帧 | 会自动丢缓冲重同步；持续出现请检查 USB 线与波特率 |
| `/leg_control/state` 的 state 一直是 0（INIT） | IMU / `joint_states` 没到，或 QoS 不匹配 | `ros2 topic hz /imu/data /joint_states`；`joint_states` 订阅是 reliable |
| 状态在 RUN/DEGRADED 之间跳 | `/robot_command` 没有稳定 50 Hz | `ros2 topic hz /robot_command`；确认 `command_param_node` 在运行 |
| 使能后停在 IDLE | 需要连续 `startup_cycles` 个健康周期（0.5 s） | 看 `state[1]` 故障位，先解决传感器超时 |
| FAULT 后关闭急停也不恢复 | 故障锁存 | 先把 `/command_param/enable` 设为 false，再重新设为 true |
| rqt 里找不到 `/command_param` | 参数节点未启动或不在同一 ROS domain | 检查 `use_command_server` 与 `ROS_DOMAIN_ID` |
| 机器人“一上电就疯转” | `theta_offset` 没标定 | 先 `enable_motor:=false` 调零 |
| 姿态符号反 | `theta_axis` 与 IMU 安装不匹配 | 切 `pitch`/`roll` 或改 `theta_offset` |
| 轮子反向 / odom 方向反 | `left/right_invert` 与实物不符 | 改 `usb_drive` 参数，上下行同时生效 |
| 一使能就往前冲 | `balance_torque_to_omega` 符号错 | 取负再试；确认 IMU 的 pitch 正方向 |
| 车不动、腿抖 | VMC `kp_x` 太大，和轮子抢位置 | 降低 `kp_x`（见第 9 章） |
| 腿朝天上“拱” | 固件 `joint_calib` 零位/方向错，或 VMC 装配分支错 | 先核对 `α=β=90°` 时足端位置，再改 `foot/front/rear_branch` |
| 仿真里腿关节不动 | actuator 是 position 不是 effort | 见 5.6 |
| 仿真里轮子越跑越快 | 浮动基座无力矩平衡，正常 | 需要闭环控制 |
| 一 launch 机器人就自由落体、落地被弹飞 6~8 m | 控制器就绪前就开始积分物理 | 开启动门控，见 **12.2** |
| spawner 报 `Switch controller timed out after 5.000000 seconds!` | 在仿真线程里查 `get_loaded_controllers()`，与 `switch_controller` 抢锁 | 就绪判定放独立线程，见 **12.2** |
| 闭环一上电就抖 / 约束力尖峰 | `connect` 的 `anchor` 坐标系搞错 | 改 `site1/site2`，见 **12.3** |
| 腿一直贴着 `[-2,2]` 限位、机器人慢慢滑走 | 目标质心高度超出五杆工作空间（`foot_y` 超过可达 0.148 m） | 降 `com_height_default`，见 **12.4** |
| 闭环下轮速发散到 100+ rad/s | 轮子惯量 3e-5，速度环纯 P 的稳定上限只有 0.03 | 轮子 `armature=2e-5`，见 **12.5** |
| 自己写 PD 试跑、`kd` 稍大就爆 | 被动杆惯量 5e-6，`kd < 2J/Δt = 5e-3` | 加 `armature`，见 **12.5** |
| 仿真里腿朝天/翻滚 | MJCF 关节方向与 LQR 假设相反（右腿整体绕 x 转 180°，左右反对称） | 见 **12.7** 改 `ctrlrange`/关节轴，或给右腿加负号 |
| 执行时间接近 2 ms | 控制线程被 IMU 回调/日志挤占 | 降 IMU 频率、少用 `ros2 param set`、关闭高频日志 |
| 拔掉 USB 后机器人继续跑 | 固件没有命令看门狗 | 上位机已发安全帧；根治要改固件 |
| `KDL does not support a root link with an inertia` | `base_link` 是根连杆且带惯量 | 预期告警，可忽略 |
| 状态永远停在 `INIT`，故障位 `0x22` 和 `0x00` 每秒来回跳 | 同 domain 里有**另一个机器人栈**在发 `/joint_states`（关节名对不上，反馈被反复判定超时） | `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1` 后重跑，见 5.6 |
| 仿真里机器人飞到 x=±100 m、z=30 m，姿态乱跳 | 同上：`/clock`、`/joint_states` 混用，`leg_control` 的 `x` 是别人的轮子 | 隔离 domain；确认 `ros2 node list` 里没有 `ros_gz_bridge` / `backend_node` |
| 站着不停前后摆 ±0.3 m，轮速 ±18 rad/s，`u` 常年贴 ±0.4 | LQR 增益比 `u_max` 大一个量级，全程 bang-bang | 增益整体缩小（1/6 那组），见 6.3 / 6.4 |
| 日志里 `FAULT 0x08` 紧跟着一条 `FAULT 0x00` | 正常：进入 FAULT 后 `controlling=false`，姿态检查被跳过，掩码重算成 0，但状态仍是 FAULT | 不用管；姿态回正后会自动回 RUN，见 11.3 |
| 大幅度推一下之后机器人被判倒伏、失力 | 瞬态角速度超过 `3 × max_pitch_rate = 9 rad/s`，视为不可救 | 想多挣扎就调大 `kFallPitchFraction/kFallRateFraction`（11.3） |
| 启动瞬态前冲（x 冲到 0.4 m） | 轮子力矩在门控放开之前就下发，挂架夹着时空转 | `wheel_arm_on_gate: true`，见 5.6 与 12.2 |

---

## 8. 已知限制

1. 模型是**平面单摆**（3 广义坐标 → 4 状态），不含 Roll / Yaw 与腿部质量分布的完整耦合。
2. 固件轮环是「位置 → 速度 → 电流」串级，LQR 的力矩只能经上位机折算成速度/角度指令下发（见 11.4）；若要真正力矩直通，需给 `Wheel_t` 增加 `torque` 字段并在固件里支持直通电流模式。
3. 关节零位由固件 `joint_calib[]` 维护，上位机只负责“关节坐标系”。
4. `gainAt` 有 1e-3 m 的缓存阈值，`quadratic` 模式本身再叠加 5e-4 死区，避免 500 Hz 下反复解 Riccati。
5. 求解器是连续时间 LQR，没有做离散化。
6. LQR 只闭环“轮子平衡”，腿的屈伸只改变 `h` 从而改变 `K`；**腿的力矩控制由 VMC 负责**（第 9 章）。
7. 关节超时未实现，若需失力保护应补到通信层；控制层的 `joint_state_timeout` 会让状态机进入 FAULT。
8. `theta_dot` 目前直接用 IMU 角速度，没有滤波。
9. 只测过数据链路；实机稳定性需要按第 6 章顺序调参。
10. `/joint_states` 里轮子的 `position` 单位是 rad；`odom` 的 `x` 单位是 m；不要混用。
11. `h` 的“实测”依赖关节零位正确，零位偏了 h 就偏，进而 K(h) 错——见 6.5。
12. `quadratic` 模式的系数只在**模型参数、Q/R 不变**时有效；改了模型或权重后需要重新拟合 `k_coeffs`。
13. 轮子控制的 LQR 模型**不含腿的 VMC 内力**；两者通过 `/joint_target`（腿）和 `/wheel_target`（轮）分开作用，是“分层”而不是“统一最优”。
14. 轮子目标 = 平衡量（左右同向）+ 行驶量（左右差速），由 `leg_control` 的分配层统一合成（平衡优先，见 11.4）。**二者共用 `max_wheel_omega` 限幅**，大速度行驶时留给平衡的余量会变小。
15. **500 Hz 是 ROS2 用户态的“软实时”**：`create_wall_timer` 受执行器调度、CPU 负载影响；真硬实时应下沉到固件。
16. **上层命令来自参数服务**：`command_param_node` 通过 ROS 2 原生参数服务接收 rqt / `ros2 param set` 的修改，不需要独立终端；launch 里的 `use_command_server` 默认 `true`。
17. `balance_torque_to_omega` 是**人为的力矩→速度折算系数**（默认 1.0 rad/s per N·m），必须按实机标定；速度域分配**不等价于**力矩域分配。
18. `FAULT` 状态下关节按“失力”处理（上位机只能发纯力矩），腿会塌下来；要保持姿态，需要给 `usb_drive` 配 `joint_kp/joint_kd`。轮子统一按“位置保持”当抱闸。**例外**：姿态越界引起、且还没到倒伏程度的 FAULT 会继续保平衡（见 11.3），这是为了让机器人有机会自己回正。
19. `/odom` 是「轮弧长 + IMU yaw」的 2D 里程计：位移只来自轮端（无滤波、打滑会漂移），朝向来自 IMU yaw（陀螺积分会缓慢漂移、磁力计易受干扰）；只用于调试/可视化，不参与平衡。
20. ~~当前仿真模型是**串联 2R**~~ → 已改：URDF/MJCF 现在都是**双五杆闭链**，与实机关节语义一致（1 个驱动髋 + 1 个被动膝/腿，见 9.5 的历史说明与附录 C）。
21. MJCF（`src/model/model/wheel_leg.xml`）已经是**双五杆闭链**（两条 `connect`），但：
    1) 无控制倒地时受「闭链 + 接触冲击」影响会被弹飞，参数调不掉，只能靠启动门控避免（见 12.8）；
    2) `leg_control` 的五杆零位与 URDF 零位差了 34°～124°（左右腿不同，右腿还反向），必须靠
       `joint_sign` / `joint_offset` 标定，否则站姿会把腿顶到限位（已实测并修复，见 12.7）；
    3) 轮速环依赖轮子 `armature`，给太小会发散（见 12.5）。

---

## 9. 足端 VMC（虚拟弹簧阻尼）

### 9.1 坐标系与机构

```
              y=0   O ------------------ D        O：近端髋（原点）
                     ╲ α            β ╱           D：远端髋 (l5, 0)
                      A                C          A = O + l1·(cosα, sinα)
                       ╲              ╱           C = D + l4·(cosβ, sinβ)
                        ╲            ╱            |AB| = l2，|CB| = l3
                             B                    B：足端（轮心）
                             ↓ y 向下
```

- 关节向量 `q = [α, β]`，每腿 2 个驱动关节：`q[0]` 对应 `leg_joint_names[0/2]`（近端/hip），`q[1]` 对应 `[1/3]`（远端/knee）。
- `α、β` 以 `+x` 为 0、转向 `+y`（向下）为正；左右腿共用同一几何，方向差异交给固件 `joint_calib`。
- 足端位置由两圆交点给出（`circle(A, l2) ∩ circle(C, l3)`），`foot_branch=+1` 取图中装配（足端在 A→C 连线下方）。

### 9.2 控制链路

```
/joint_states ──► 正运动学 T(q) ──► x（足端位置）
                       │
                       └─ 雅可比 J(q) ──► ẋ = J·q̇（足端速度）

    F = Kp·(x_d − x) + Kd·(ẋ_d − ẋ)        ← 两根虚拟弹簧 + 两个阻尼器（逐轴）
    τ = Jᵀ·F                                ← 虚功原理，映射回两个关节力矩
                     │
                     ▼
              /joint_target (effort)
```

雅可比是**解析解**，不是数值差分：把两连杆的长度约束 `|B−A|=l2`、`|B−C|=l3` 对 `q` 求导

```
[ (B−A)ᵀ ]        [ (B−A)ᵀ·∂A/∂α      0          ]
[ (B−C)ᵀ ] · J =  [     0         (B−C)ᵀ·∂C/∂β   ]
```

左端矩阵可逆时就能直接解出 `J`（两连杆共线时奇异，节点会把该腿力矩置零）。

其它映射：
- 逆雅可比：`q̇ = J⁻¹·ẋ`（2 DOF/腿，`J` 是 2×2 方阵，直接求逆，不需要 `(JᵀJ)⁻¹`）
- 逆运动学：`q = T⁻¹(x)`，每腿 4 组装配解，用分支参数或 `q_seed` 选一组

### 9.3 `leg_control_params.yaml` 速查

| 组 | 参数 | 默认 |
|---|---|---|
| 几何 | `l1` / `l2` / `l3` / `l4` / `l5` | 0.050 / 0.105 / 0.105 / 0.050 / 0.060（URDF 复算，见 12.7） |
| 装配 | `foot_branch` / `front_branch` / `rear_branch` | 1 / 1 / 1 |
| 刚度 | `kp_x` / `kp_y` | 60 / 600 N/m |
| 阻尼 | `kd_x` / `kd_y` | 3 / 40 N·s/m |
| 限幅 | `force_max_x` / `force_max_y` | 40 / 150 N |
| 目标 | `foot_target_x` / `com_height_default` / `com_height_rate` | 0.03 m / 0.0834 m / 0.30 m·s⁻¹ |
| 输出 | `joint_torque_max` / `joint_torque_sign` | 8 N·m / 4×1.0 |
| 标定 | `joint_sign` / `joint_offset` | [1, 1, -1, -1] / 见 12.7 表（实机已标定） |
| 接口 | `joint_states_topic` / `joint_target_topic` / `leg_joint_names` | `joint_states` / `joint_target` / 4 个腿关节名 |

除话题名外都支持 `ros2 param set` 热更新（几何非法、`kp/kd<0`、`joint_torque_max<=0` 会被拒绝并返回原因）。

### 9.4 调参顺序

```
① 量 l1..l5（实测）→ 确认 α=β=90° 时足端位置与实机一致（启动日志会打印）
② 标定关节零位：量曲柄零位角，写进 `joint_offset`（右腿还要 `joint_sign=-1`），
   验收标准 = 控制器 FK 的足端与模型/实机差 < 1 mm（见 12.7 的对拍表）
③ kp_y 从小到大（100 → 300 → 600），吊起机器看腿能否"撑住"
④ kd_y 抑制上下弹跳（约 0.2·sqrt(kp_y·m_单腿) 起调）
⑤ kp_x/kd_x 抑制前后晃动（一般比 y 轴软，避免和轮子 LQR 抢位置）
⑥ 力矩方向：若某关节"越用力越往错误方向走"，把 joint_torque_sign 对应位置 -1
⑦ 最后再放开 force_max / joint_torque_max
```

**别和 LQR 抢**：VMC 管腿（`/joint_target` 的 effort），LQR 管轮（`/wheel_target`）。`kp_x` 太大时腿会把机身前后"锁死"，轮子的平衡动作会被抵消，表现为车不动、腿抖。

---

### 9.5 五杆闭链 vs 串联 2R（历史清单）

> **⚠ 已过时，留作记录。** 当前 `src/model/model/{wheel_leg.urdf,wheel_leg.xml}` 已经是**双五杆闭链**
> （4 个驱动髋 `lf/lb/rf/rb_joint1` + 4 个被动膝 + 2 条 `connect`，actuator 全是 `motor`），
> 详见 12.3 / 12.7 与**附录 C**。下表是当年"串联 2R → 五杆"的差异核对，改模型时仍有参考价值。

实机是**五杆闭链**（每腿 2 个驱动髋关节，膝盖是被动铰点），而当时的
`src/model/model/{wheel_leg.urdf,wheel_leg.xml}` 是**串联 2R**（`base_link → 大腿 → 小腿 → 轮`）。
两者不能混用，用仿真验证 VMC 前必须先对齐：

| 项 | 五杆（实机 / VMC） | 串联 2R（旧 URDF/MJCF） | 影响 |
|---|---|---|---|
| 驱动关节 | 2 个髋关节 `α, β` | 髋 + 膝 `q1, q2` | 关节语义不同，`/joint_target` 不能直接互换 |
| 足端位置（中位） | `α=β=90°` → `(0.030, 0.1506)` m（按当前 l1..l5） | `q1=q2=0` → `(0, 0.30)` m | 足端定义与腿长语义不同 |
| 腿长 | `y_foot`（足端 y 坐标） | `l1·cos(q1) + l2·cos(q1+q2)` | LQR 的 `h` 公式只对串联模型成立 |
| 关节限位 | 摆臂 ±90° 附近 | 髋 ±1.30 / 膝 [-2.60, 0.10] rad | 直接搬限位会撞机构 |
| 质量分布 | 摆臂 + 连杆 | 大腿片 + 小腿片 | 惯量矩阵不同，VMC 增益需重标 |

核对方法（进仿真前先跑一遍）：

1. 用 `foot_vmc` 的正运动学，在 `α=β=90°` 处算足端位置，和实机卷尺量出的值对比（误差应 < 5 mm）。
2. 把实机 `q = [α, β]` 代进串联模型的 `h` 公式，看是否仍落在 `[h_min, h_max]`——差得多说明
   LQR 的高度估计不能直接用于实机，需要改成用 VMC 足端 y 反算 `h`。
3. 要在 MuJoCo 里跑 VMC，得先把 MJCF/URDF 改成双髋闭链，并把 `position` actuator 换成 `motor`。

---

## 10. 差速底盘运动学（v, ω → 左右轮速）

### 10.1 公式与符号

```
        y（车头方向）
        ↑
   ω_L ─┤├─ ω_R        左轮在 +y 侧，右轮在 −y 侧，轮距 L，轮半径 r
        └───► x
```

| 量 | 含义 | 单位 |
|---|---|---|
| `v` / `linear.x` | 底盘中心前进线速度 | m/s |
| `ω` / `angular.z` | 底盘 yaw 角速度（转向角速度） | rad/s |
| `v_L` / `v_R` | 左/右轮线速度 | m/s |
| `ω_L` / `ω_R` | 左/右轮角速度（= 轮速 ÷ r） | rad/s |
| `L` | 轮距（`wheel_separation`） | m |
| `r` | 轮半径（`wheel_radius`） | m |

**正解**（轮速 → 底盘，差速运动学；`/odom` 现改为「轮弧长 + IMU yaw」，见第 11 章）：

```
v = (ω_L + ω_R)·r / 2
ω = (ω_R − ω_L)·r / L
```

**反解**（底盘 → 轮速，由控制核心的分配层执行）：

```
ω_L = (v − L/2·ω) / r
ω_R = (v + L/2·ω) / r
```

符号：`ω > 0` 左转（逆时针、yaw 增大，右轮快左轮慢）；`ω < 0` 右转；
`v = 0、ω ≠ 0` 是原地转；`ω_L = ω_R` 是直线。

限幅顺序：先把 `v`、`ω` 截到 `max_linear_velocity` / `max_yaw_rate`，再算轮速；
然后与平衡分量相加，最后整体截到 `max_wheel_omega`（平衡优先，见 11.4）。

### 10.2 参数速查（`leg_control_params.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `wheel_separation` | 0.305 | L [m]，唯一来源（分配层与 `/odom` 共用）；URDF 两轮轴 y=±0.1525，按轮胎接地中心线约 0.297 |
| `wheel_radius` | 0.030 | r [m]，唯一来源；STL 轮胎外缘 0.0299 |
| `max_linear_velocity` / `max_yaw_rate` | 1.0 / 3.0 | 车体线速度 / 角速度限幅 |
| `max_wheel_omega` | 20.0 | 单轮角速度总限幅 [rad/s] |
| `max_drive_omega` | 12.0 | 行驶分量限幅 [rad/s] |
| `command_timeout` / `command_fault_timeout` | 0.3 / 1.0 s | 命令超时降级 / 长期丢失停机 |
| `control_rate` | 500 | 控制环频率 [Hz] |

均支持 `ros2 param set /leg_control ...`。

### 10.3 与平衡控制的关系

```
轮子目标速度 = 平衡分量（LQR，左右同向） + 行驶分量（差速，左右不同）
```

- 只有平衡时（命令里 `v=ω=0`），左右轮只随姿态修正。
- 两者都在时就是"边走边自平衡"：平衡量消除俯仰，行驶量负责前进/转向。
- 命令超时 → 行驶分量归零（`DEGRADED`），保留平衡；长期丢失 → `FAULT`。
- 使用前确认 `/odom` 的 `x` 方向与实物一致（推车向前 `x` 增大），否则 `left/right_invert` 反了，
  差速的转向也会跟着反。

### 10.4 最小闭环（可选）

分配层的差速部分是**开环运动学**。要压住直线漂移，可以再加一个轮速环：
订阅 `/joint_states` 里两个轮子的 `velocity`，用 PI 修正 `v_d`，或直接用
`/robot_command` 的 `v` 与 `/odom.twist` 的偏差修正。这一步没做，接口已经留好
（`wheel_target` 就是轮速指令）。

> 独立的 `diff_drive_node` 已移除：差速运动学由 `leg_control` 的分配层直接调用，
> 指令来源是 `/robot_command`（50 Hz 参数服务）而不是 `/cmd_vel`。

---

## 11. 分层控制架构（500 / 200 / 50 Hz）

### 11.1 原则

1. **实时控制环独立、固定周期**：500 Hz 任务只做“读快照 → 算 → 发”，不等参数服务、不等 USB、不做日志 IO。
2. **上层只改目标**：50 Hz 参数服务节点只发布意图（速度、转向、质心高度、模式），不参与任何控制计算。
3. **通信只搬运**：200 Hz USB 节点把“最近一帧命令”搬到下位机，把状态帧搬上来；包到达时刻不决定控制何时运行。
4. **三个频率不需要对齐**：一个 20 ms 的参数命令会被 10 个 2 ms 控制周期复用，中间靠“最近有效命令 + 超时门限”衔接。
5. **参数不在控制环中途改**：参数回调只校验并放进待生效区，下一个周期边界统一切换（`applyConfig`）。

| 模块 | 频率 | 周期 | 代码位置 |
|---|---|---|---|
| 状态快照 + 状态机 + Pitch LQR + 双 VMC + 轮子分配 | 500 Hz | 2 ms | `leg_control_node` / `leg_control_core` |
| USB CDC 收/发 + 链路看门狗 | 200 Hz | 5 ms | `usb_drive_node.cpp` |
| 参数服务命令 | 50 Hz | 20 ms | `command_param_node.cpp` |
| 下位机轮子 PID / 关节运控 | 333 Hz（3 ms） | 3 ms | `stm32/one/task/task.c` |
| 下位机状态上报 | 100 Hz（10 ms） | 10 ms | 同上 |

### 11.2 一个控制周期里发生什么

```
① 若参数待生效 → applyConfig()               （只在周期边界）
② 组装健康快照 → SafetySupervisor::update()   （状态机推进）
③ 若允许出平衡：
     h = h(4 关节)  →  K = K(h)  →  u = −K(x − x_ref)  限幅 u_max
     左腿 VMC / 右腿 VMC（各自 q、q̇，共用几何与增益）→ τ0..τ3
     轮子分配：ω_ref = clamp(平衡分量 + 行驶/转向分量) 并按优先级限幅
④ 发布 /joint_target（effort=τ）、/wheel_target（ω_L, ω_R）、/odom
⑤ 统计本周期执行时间与周期抖动
```

- **快照一致性**：单线程执行器下，IMU/命令回调和定时器串行，读到的一定是“某一时刻的完整组合”。
- **姿态来源**：`theta = rpy(axis) − theta_offset`，`theta_dot = ω_y`；`theta_axis` 可切 `pitch/roll`。
- **质心高度目标**：命令里的 `com_height` 按 `com_height_rate` 斜坡逼近（默认 0.30 m/s），避免参数突变就给 VMC 一个阶跃目标。
- **代码分层**：`types`（数据）→ `core`（控制计算，无 ROS）→ `io`（消息解析与打包）→ `node`（参数、话题、定时器）。
  核心可以直接在单测里喂 `StateSnapshot`，不需要起 ROS。

### 11.3 状态机与安全监督

```
        ┌──────┐  enable=0   ┌──────┐  连续 startup_cycles 健康 + enable  ┌─────┐
        │ INIT │────────────►│ IDLE │───────────────────────────────────►│ RUN │
        └──────┘             └──────┘                                    └──┬──┘
            ▲                     ▲           命令超时 → 降级                 │
            │                     │        ┌──────────┐  命令恢复            │
            │                     │        │ DEGRADED │◄─────────────────────┘
            │                     │        └────┬─────┘
            │                     │             │ 硬故障
            │                     │        ┌────▼─────┐
            └─────────────────────┴────────│  FAULT   │
                    故障消失且 enable=0     └──────────┘
```

姿态类故障（`0x08`）额外有一条 `FAULT → RUN` 的自动恢复边：姿态回到门限 75% 以内、连续 10 ms 健康即回 `RUN`，
不需要撤 `enable`；只有其它硬故障才必须手动复位。

| 状态 | 轮子 | 关节 | 说明 |
|---|---|---|---|
| `INIT` | 位置保持 | 失力 | 上电自检，传感器没就绪就停在这里，不报故障停机 |
| `IDLE` | 位置保持 | 失力 | 等使能；`startup_cycles` 连续健康后才允许进 RUN |
| `RUN` | 平衡 + 行驶/转向 | VMC 力矩 | 正常控制 |
| `DEGRADED` | **速度目标归零**，仍保留平衡 | VMC 力矩 | 上层命令超时（默认 0.3 s） |
| `FAULT` | 位置保持（当抱闸） | 失力 | IMU/关节超时、姿态越界、急停、命令长期丢失（>1 s） |
| `FAULT`（姿态类，未倒伏） | **继续平衡 + 位置保持** | **VMC 力矩** | 只有姿态越界、且没到倒伏程度：继续保平衡等它自己回正，见下 |

故障位（`/leg_control/state[1]`，按位或）：

| 位 | 名称 | 触发条件 |
|---|---|---|
| `0x01` | IMU | 未收到或年龄 > `imu_timeout` |
| `0x02` | 关节反馈 | 未收到或年龄 > `joint_state_timeout` |
| `0x04` | 命令 | 命令年龄 > `command_fault_timeout` |
| `0x08` | 姿态 | `abs(pitch) > max_pitch` 或 `abs(pitch_dot) > max_pitch_rate` |
| `0x10` | 急停 | 参数 `estop=true` / 命令 `estop=1` |

五条容易踩的规则：

- **故障不是一律“零力矩”**：轮子直接撤掉平衡力矩可能让车直接倒；所以 FAULT 里轮子改成“位置保持”（等价抱闸），关节才失力。
- **姿态类故障（`0x08`）是可自动恢复的**（2026-10-30 改）：因为 FAULT 里关节一失力，机器人 0.1 s 内
  就会真的倒下去，根本没有恢复窗口。现在只要**还没到倒伏**，FAULT 期间仍照常输出平衡 / VMC 力矩，
  姿态回到门限 **75%** 以内、且角度和角速度都收敛、连续 5 个周期（10 ms）→ 自动回 `RUN`。
  实测一次 `0x08` 到 `RUN` 约 20 ms。
- **其它硬故障仍然锁存**：撤 `enable` 再重新使能才能回 `IDLE`（防止机器人“自己站起来”）。
- **倒伏判据**：`|pitch| > 2.5 × max_pitch`（0.35 → 0.875 rad ≈ 50°）或
  `|pitch_rate| > 3 × max_pitch_rate`（3.0 → 9 rad/s）时认定已经救不回来，才真正失力。
  两个倍数在 `control_state.hpp` 的 `kFallPitchFraction / kFallRateFraction` 里，没有做成 ROS 参数。
- 需要故障时腿部保持姿态：给 `usb_drive` 配 `joint_kp` / `joint_kd`（固件运控帧的 kp/kd 由 `usb_drive` 填），否则上位机只能给纯力矩。

### 11.4 轮子控制与分配规则

```
u            = −K(x − x_ref)                         ① Pitch LQR，单位 N·m
ω_balance    = clamp(u · balance_torque_to_omega)    ② 力矩 → 速度域折算（实机标定）
v_L, v_R     = v_d ∓ L/2·ω_d                         ③ 差速运动学
ω_drive      = clamp((v_L, v_R) / r)                 ④ 行驶分量限幅
ω_ref        = clamp(ω_balance + ω_drive, ±max_wheel_omega)   ⑤ 平衡优先，整体限幅
```

为什么停在“速度域”：下位机 M2006 用的是 **位置 → 速度 → 电流** 串级 PID（`wheel_control_task`），
USB 帧里的 `Wheel_t` 只有 `{angle, omega}`，**没有力矩字段**，所以上位机没法直接给轮子力矩。
LQR 的 `u` 只能经 `balance_torque_to_omega` 折算成速度目标，再由固件速度环跟踪。
要真正在力矩域分配，需要给协议加 `torque` 字段并让固件支持直通电流模式。

限幅优先级：`u_max`（单轮平衡力矩）→ `max_balance_omega`（平衡分量）→ `max_drive_omega`（行驶分量）→ `max_wheel_omega`（总和）。

### 11.5 实时性怎么验证

`/leg_control/timing` 每秒发 5 个数：周期均值、周期最大、执行均值、执行最大（ms）、超期次数
（“超期”= 周期 > 1.5 倍名义周期）。执行最大超过 2 ms 时节点会打 WARN。

```bash
ros2 topic echo /leg_control/timing
# 实测（本机无硬件、仅假数据）：周期 2.000 / 最大 2.32 ms，执行 0.018 / 最大 0.078 ms
```

要点：

- **不能只在循环末尾 delay 2 ms 就算 500 Hz**：要看“周期分布 + 最大执行时间 + 超期计数”。
- `rclcpp` 的 `create_wall_timer` 运行在 ROS 执行器里，IMU 200 Hz、命令 50 Hz 的回调会和控制周期共享线程；
  实机上如果执行最大时间接近周期，先把 IMU 频率降到 200 Hz、把日志/参数回调移出控制线程。
- 平均周期稳定但“最大值”偶尔超过 2 ms，通常是节点启动首帧（首次 Riccati 求解 / 首次内存分配），属正常。

### 11.6 与下位机（`stm32/one`）的分工

| 层 | 运行位置 | 干什么 | 对应固件/节点 |
|---|---|---|---|
| 上层命令 | 上位机 50 Hz | 把 rqt / ros2 param 参数变成目标（速度、转向、质心高度、使能、急停） | `command_param_node` |
| 状态估计 | 上位机 500 Hz | 关节角 → h；IMU 四元数 → θ 与 yaw；轮弧长 + yaw → 2D 里程计 | `leg_control_node` |
| 平衡 / VMC / 分配 | 上位机 500 Hz | LQR、双腿 VMC、轮子速度域分配 | `leg_control_core` |
| 通信 | 上位机 200 Hz | 96 B 目标帧下行 / 208 B 状态帧上行、链路看门狗 | `usb_drive_node` |
| 电机环 | STM32 333 Hz | M2006 位置-速度串级 PID、RobStride 运控帧、CAN 收发 | `wheel_control_task` / `joint_control_task` |
| 状态上报 | STM32 100 Hz | 打包 `RobotState_t` 并经 USB 发出 | `state_send_task` |

关键约束：

- 固件按**结构体裸内存**收发，`data_pack.h` 与 `task.h` 必须逐字节一致（编译期 `static_assert` 会先拦）。
- 固件的关节坐标是**电机坐标系**，零位/方向由 `joint_calib[]` 处理；上位机看到的已经是关节坐标系。
- 固件**没有命令超时**：USB 断开后它会继续执行最后一帧。上位机的兜底是“状态帧超时 → 发安全帧”，
  但这依赖 USB 仍然通；真正的断线安全应在固件里补 `target_age` 看门狗。
- 固件 4 个任务优先级都是 1，靠 `vTaskDelayUntil` 定节拍；如果后面要在固件里加 500 Hz 控制环，
  建议合并成一个控制任务顺序执行，而不是给每个控制器各开一个任务。

### 11.7 建议的开发顺序

1. **驱动与标定**：六个电机的正方向、编码器单位、IMU 姿态定义、力矩单位、CAN 限幅。
2. **单腿 VMC**：撑住车身，先核对 `α=β=90°` 的足端位置，再放开 Kp/Kd（第 9 章）。
3. **Pitch LQR**：先用单元测试和模型仿真验证，再实机加保护支架（第 6 章）。
4. **轮子速度与转向**：确认 `balance_torque_to_omega` 的符号与量级，避免“越平衡越倒”。
5. **分层合环**：`wheel_leg_control.launch.py` 全开，观察 `/leg_control/state` 的 `state` 与 `faults`。
6. **安全测试**：拔 USB、断电、越界推倒、触发 `estop`，确认状态机进入 `DEGRADED` / `FAULT` 且机器人不会突然跌落。
7. **实时性回归**：记录 `/leg_control/timing` 的周期最大、执行最大、超期次数，作为每次改动的基线。

---

## 12. MuJoCo 双闭链仿真：启动门控与闭链稳定性

这一章记录 2026-10 那次排查的**完整结论**：现象、根因、改法、实测数据、以及"什么能修好、什么修不好"。
所有数字都来自本机实测（MuJoCo 3.2.7、`scene.xml` + 500 Hz `leg_control` 全链路），改完可以直接对照复现。

### 12.1 症状总览

| 症状 | 真正的根因 | 结论 |
|---|---|---|
| 一起 launch 机器人就自己往下掉，落地后被弹飞 6～8 m，`/joint_states` 里轮子转到几十 rad | `mujoco_ros2_control` 构造时先 `mj_step()`、`update()` 一开始就积分，而控制器要到 0.3 s 后才 active | **启动门控**（12.2） |
| 控制器 spawner 报 `Switch controller timed out after 5.000000 seconds!`，全部激活失败 | 门控在**仿真线程**里调 `get_loaded_controllers()`，和 `switch_controller` 抢锁 | 就绪判定必须放独立线程（12.2） |
| 两条 `connect` 闭链一上电就抖、约束力尖峰 | `anchor` 是按 body2 局部系解释的，手写数值容易差一个坐标系 | 改用 `site1/site2`（12.3） |
| 腿一直贴着 `[-2, 2]` 关节限位、机器人慢慢滑走 | **目标**质心高度超出五杆工作空间（不是初始姿态的问题） | `com_height_default` 0.18 → **0.0834**（12.4） |
| 闭环下轮速仍然发散到 100+ rad/s | 轮子惯量 3e-5，速度环纯 P 的稳定上限只有 0.03 | 轮子加 `armature="2e-5"`（12.5） |
| 用普通 PD 试跑就爆，`kd=0.5` 完全不行 | 被动杆惯量 5e-6，500 Hz 显式 PD 要求 `kd < 2J/Δt = 5e-3` | 被动关节加 `armature="2e-4"`（12.5） |
| 机器人倒地瞬间速度冲到几十 m/s、飞出 8 m | MuJoCo「闭链 + 接触冲击」的固有问题，参数调不掉 | 只能靠门控/控制器避免倒地（12.8） |
| 故障位 `0x22 ↔ 0x00` 每秒抖、状态永远 `INIT`，偶尔飞出去 z=32 m / x=−124 m | 同一 ROS domain 里**还有另一套机器人栈**在发 `/joint_states`、`/clock` | 隔离 `ROS_DOMAIN_ID`（5.6、12.11） |
| 能站但前后摆 ±0.3 m、轮速 ±18 rad/s、`u` 常年贴 ±0.4 | LQR 增益比 `u_max` 大一个量级，全程 bang-bang（极限环） | 增益缩到 1/6（6.3、12.11） |

### 12.2 启动门控：控制器没就绪之前不要跑物理

#### 为什么

`mujoco_ros2_control::update()` 的原始结构是"到点就 `mj_step()`"，
而 `leg_effort_controller` / `wheel_effort_controller` 要等 spawner 服务调用完成才 active。
这中间 0.3～1 s 里机器人是**完全无力矩的自由落体**：从 0.16 m 高砸到地面、两个轮子同时接触，
本来就是"两轮平衡车被扔在地上"，然后被接触求解弹飞。这不是模型不稳定的问题，是**时序**问题。

#### 怎么实现

参数（`mujoco_ros2_control_parameters.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `start_gate_enabled` | `true` | 总开关 |
| `start_gate_controllers` | `[joint_state_broadcaster, leg_effort_controller, wheel_effort_controller]` | 必须全部 active |
| `start_gate_hold` | `1.0` | 全部 active 后再保持多久 [s，仿真时间] |
| `start_gate_timeout` | `20.0` | 最长等待，超时强制放开并告警 |

launch 参数：`start_gate:=true`、`start_gate_hold:=1.0`（`wheel_leg_mujoco_sim.launch.py`）。

挂架的做法是**"竖直导轨"**：门控期间照常 `mj_step()`，但每步把机身（`free joint`）的
`x / y / 姿态`按回初始值、**竖直方向放开**：

```cpp
// step_simulation()：门控没放开时
mj_step(mujoco_model_, mujoco_data_);
clamp_base_to_hold_pose();          // qpos[0..1]、qpos[3..6] 复位；qvel[0..1]、qvel[3..5] 清零
mj_forward(mujoco_model_, mujoco_data_);
update_start_gate();
```

为什么不干脆"整机冻结、不积分"？试过，不行：
冻结时控制器指令一直在涨、状态却永远是 0（轮速环积分饱和），放开那一瞬间姿态角速度冲到 4.5 rad/s，
直接触发 `leg_control` 的 `max_pitch_rate`（当时 6.0，现在是 3.0）→ `FAULT 0x08`。
改成导轨挂架后腿和轮子都在真实运动，放开时已经在稳态，**故障消失**。

#### 轮子力矩必须和门控同一时刻放开

`sim_bridge` 订阅门控的 latched 话题 `/mujoco/start_gate_open`，`wheel_arm_on_gate: true` 时只有门控
真正放开的那个周期才开始把 `/wheel_target` 当作轮子力矩下发。以前用固定延时 `wheel_start_delay=1.45 s`，
而门控放开时刻在 **1.12～1.60 s** 之间浮动（取决于 spawner 何时把三个控制器激活完），就会出现
“挂架还夹着、轮子已经转起来”。实测启动瞬态 x 一度冲到 **0.44 m**；改成跟门控同步后只有
x∈[−0.007, 0.055]、θ∈[−0.017, 0.003]，而且 `u` 全程不饱和。

#### 关键坑：就绪判定不能在仿真线程里做

`ControllerManager::switch_controller()` 会持锁等 `update()` 把切换做完，
而 `get_loaded_controllers()` 要拿同一把锁 —— 在仿真线程里调用就是自己等自己，实测表现为：

```
[controller_manager]: Switch controller timed out after 5.000000 seconds!
[spawner]: Failed to activate controller : joint_state_broadcaster
```

所以轮询放在独立线程（`start_gate_poll_loop()`，100 ms 一次），仿真线程只读一个
`std::atomic<bool>`。这是本次最隐蔽的一个 bug，改别的东西时不要把它挪回主循环。

#### 实测时序

```
启动门控已启用：等 [joint_state_broadcaster, leg_effort_controller, wheel_effort_controller] 全部 active 后再保持 1.00 s
控制器已全部 active（sim t=0.360），挂架保持 1.00 s 后放开物理步进
leg_control: 状态 RUN，故障 0x00          ← 释放之前已经进 RUN
启动门控放开：开始物理步进（sim t=1.364，挂架保持 1.00 s）
```

门控期间 `/clock`、`/joint_states`、IMU 都正常发布（`leg_control` 的自检要靠它们），
关节速度在 1e-3 量级、机身不动。

#### 用法

```bash
# 正常（平衡使能）
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false enable:=true
# 想观察挂架状态（不使能，机器人被夹着）
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false
# 临时关掉门控（不推荐，会复现自由落体）
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py start_gate:=false
```

### 12.3 闭链怎么定义才不出错：`anchor` → `site1/site2`

原来的写法：

```xml
<connect name="left_close" body1="lb_link2" body2="lefoot_link"
         anchor="0.004200 -0.104800 -0.010900" .../>
```

问题：`anchor` 是**在 body2 的局部系里**解释的。手写 CAD 量出来的数字，只要坐标系记错，
初始就带一截约束误差，一上电就被约束力拽。本例里那个数字其实是"轮轴在 `lb_link2` 系下的坐标"，
差 1.9 mm，属于碰巧还能跑；以后改任何一个连杆尺寸都会静默把闭环拉开。

改成 site 之后，闭合点看得见、也算得出来：

```xml
<!-- lb_link2 里 -->
<site name="lb_rocker_axle" pos="0.004066 -0.105117 -0.012800" size="0.004" rgba="1 0.2 0.2 0.6"/>
<!-- lefoot_link 里（轮轴中心） -->
<site name="lefoot_axle" pos="0 0 0" size="0.004" rgba="1 0.2 0.2 0.6"/>

<connect name="left_close" site1="lb_rocker_axle" site2="lefoot_axle"
         solref="0.01 1" solimp="0.9 0.95 0.005"/>
```

site 的数值用 FK 反算（`qpos=0` 时把轮轴世界坐标转到摇杆局部系）：

```
lb_link2 ← lefoot_link 原点:  0.004066  -0.105117  -0.012800
rb_link2 ← rifoot_link 原点:  0.042835   0.095870   0.001204
```

验证（改完必须过这一关）：

```bash
qpos=0 闭环残差 = 3.812e-07 m      # 必须 ~1e-7；> 1e-4 说明 site 放错了
```

`solref=0.01` 而不是 `0.006`：`dt=0.002` 时前者是 5 个步长、后者只有 3 个，略软一点更不容易在冲击下炸。
正常站立时残差仍是 1e-6 量级，只有用 3 N·m 猛甩整条腿才会看到 3.6 mm / 64 N 的瞬时值。

### 12.4 五杆的奇异点：初始姿态没问题，**目标姿态**有问题

#### 初始姿态不是奇异点

奇异位形 = 曲柄与连杆共线（夹角 0° 或 180°）。`qpos=0` 实测：

```
左腿两连杆夹角 76.15°   右腿 65.13°     ← 离奇异点很远，初始姿态是安全的
```

#### 但控制器的默认目标高度**超出了工作空间**

| 量 | 数值（URDF 复算 / 仿真实测） |
|---|---|
| 曲柄 `l1`（髋→前肘） | 0.0500 m |
| 连杆 `l2`（前肘→轮轴，平面投影） | 0.1050 m |
| `l1 + l2`（最大伸展） | 0.1550 m |
| 两髋间距 `l5` | 0.0600 m |
| **可达 `foot_y` 上限**（零重力摆动 16 s 实测） | **0.1483 m**（= 0.96 × (l1+l2)） |
| home 姿态的 `foot_y`（轮轴 z 0.0321 − 髋 z 0.1493） | 0.1172 m（79% 行程，很舒服） |
| 曾经的 `com_height_default=0.18` 对应 `foot_y = 0.18 + 0.0338` | **0.214 m → 144% 上限** |

也就是说 VMC 在命令腿伸到**比机构最大伸展还长**的位置，腿只能顶到 `[-2, 2]` 限位，
表现为"腿贴着限位、机器人慢慢滑走"。现在 `com_height_default` 与 `command_param.com_height`
都取 **0.0834**（换算 `foot_y = 0.0834 + 0.0338 = 0.1172`，正好等于 home 姿态）：

```yaml
leg_control:
  ros__parameters:
    com_offset: -0.0338        # home 姿态摆杆质心在髋轴下方 33.8 mm（URDF）
    com_height_default: 0.0834 # foot_y = 0.1172 m，与 home 姿态一致
command_param:
  ros__parameters:
    com_height: 0.0834
```

自查方法：`foot_y = com_height − com_offset` 应远离伸直奇异位形（`com_height_max=0.100` → `foot_y=0.1338`，
是上限的 90%，节点启动时会用 11 个采样高度校验 `脚端距离 < 0.9·(l1+l2)`）；
`com_offset` 按 home 姿态下排除两轮的摆杆质心计算，改变腿姿后仍需复核该近似。

### 12.5 被动关节必须给 `armature` 和阻尼

MuJoCo 里惯量小于 1e-5 kg·m² 的连杆，配合 500 Hz 的**显式** PD/阻尼，是典型的发散组合。
稳定性条件是 `kd < 2·J/Δt`：

| 对象 | 网格惯量 J | `dt=0.002` 时允许的最高增益 | 加 armature 后 | 处理 |
|---|---|---|---|---|
| 4 个髋 | 4.6e-6 | 4.6e-3（`kd=0.5` 必爆） | `armature=2e-4` → 0.2 | 见下 |
| 4 个被动杆 | 4.6e-6 ～ 5.7e-6 | 同上 | 同上 | `armature=2e-4` |
| 2 个轮 | 2.3e-5 / 3.0e-5（左/右 izz） | **kp < 2J/Δt = 0.03**，而 sim_bridge 用的 0.02 太贴边 | `armature=2e-5` → 0.05 | 见下 |

最终参数（`wheel_leg.xml`）：

```xml
<!-- 髋（驱动） -->
<joint name="lf_joint1" type="hinge" ... damping="0.05" armature="2e-4" frictionloss="0.01" .../>
<!-- 被动杆 -->
<joint name="lf_joint2" type="hinge" ... damping="0.05" armature="2e-4" frictionloss="0.005" .../>
<!-- 轮 -->
<joint name="lefoot_joint" type="hinge" ... damping="0.02" armature="2e-5" frictionloss="0.002"/>
```

**轮子的 armature 千万别加大**。轮半径只有 0.03 m，`armature` 会按 `J/r²` 变成等效平动质量：

```
armature 2e-5 → 0.022 kg（可忽略，当前值）
armature 3e-3 → 3.3 kg     （leg_control 里 cart_mass 才 0.285 kg，直接失真 10 倍以上）
```

实测对比：轮子完全不给 armature 时，闭环跑 6 s 内轮速发散到 **121 rad/s**；
给 2e-5 后轮速稳定在 12.6 rad 附近不再增长。

### 12.6 两条闭链会不会和接触形成过度约束

**不会**，可以数自由度验证：

```
每条腿 5 个 hinge − 2 条有效约束（平面五杆）= 3 DOF
（2 个机构自由度 + 1 个绕轮轴的轮子自转）
```

`connect` 给的是 **3 行**约束，其中沿铰链轴方向（base 的 z 轴）那一行是**相容的冗余**：
本模型所有 hinge 轴都平行，机构是平面的，那行约束恒为 0、不做功。
所以不是"过度约束导致打架"，把 `condim` / 摩擦 / `base_col` 去掉都试过，现象完全不变。

全工作空间摆动扫描（零重力、无接触、3 N·m 猛甩 4 s）：
残差最大 3.6e-3 m、约束力最大 64 N，始终能闭合 —— 机构本身是自洽的。

### 12.7 左右腿是反对称的（关节轴 + 符号）

右腿在 URDF/MJCF 里相对左腿整体绕 **x 轴多转了 180°**（髋 origin 的 `rpy`：左 `−1.5708 0 0`、
右 `+1.5708 0 0`），而所有关节轴都写 `0 0 -1`，于是：

| 施加 | 关节转角 | 轮轴世界位移 |
|---|---|---|
| `lf_joint1` +0.2 N·m | +0.190 rad | (+0.0037, +0.0019) |
| `rf_joint1` +0.2 N·m | +0.180 rad | **(−0.0032, −0.0017)** |

同一个正号力矩，左右腿往**相反方向**走。所以：

- 任何"左右腿共用一套五杆 FK/IK"的代码，必须对右腿的关节角/力矩取反，或者统一换算到 base 系再算；
- 判据：静止站姿下应该是 `rf_joint1 ≈ −lf_joint1`、`rb_joint1 ≈ −lb_joint1`。
  实测站姿 `lf=-0.98 / lb=-0.31 / rf=+1.27 / rb=+0.26`，符号是对的。

#### 关节角 → 五杆角标定（已实测并修复）

曲柄在 URDF/MJCF 里的**零位不指向五杆图里的 α=β=0 方向**，所以 `leg_control` 不能把
`/joint_states` 的角度直接当 α/β 用。测量方法：在 MuJoCo 里量「髋心→肘心」在五杆平面
（x 从 O 指向 D、y 向下）的方向角 `atan2(dy, dx)`，q=0 与 q=±0.4 线性拟合（斜率恰为 ±1，残差 1e-3 度），
**斜率 = `joint_sign`、截距 = `joint_offset`**。同一套数也可以直接从 URDF 关节 origin 算出来：

| 关节 | 髋→肘（五杆平面） | 零位角 = `joint_offset` | `joint_sign` |
|---|---|---|---|
| `lf_joint1` → α_L | (−0.0275573, +0.0417204) | **+123.446°** = 2.154534 rad | +1 |
| `lb_joint1` → β_L | (+0.0412990, +0.0281850) | **+34.312°** = 0.598858 rad | +1 |
| `rf_joint1` → α_R | (−0.0249005, +0.0433586) | **+119.868°** = 2.092103 rad | −1 |
| `rb_joint1` → β_R | (+0.0282360, +0.0412640) | **+55.617°** = 0.970701 rad | −1 |

即 `α/β = joint_sign·q + joint_offset`（`leg_control_params.yaml` 里同名参数，见 `leg_control_core`
的 `legJointAngles()`；`τ_q = joint_sign·τ_α`）。右腿取 −1 就是因为上面那条「整体绕 x 轴转 180°」。

> 精度说明：β_L / α_R / β_R 三个数与 URDF 复算逐位一致；α_L 的 yaml 现值是 2.155471（=123.499°），
> URDF 精确值是 **2.154534（=123.446°）**，差 0.054°（5 cm 曲柄上 0.05 mm 足端位移）。
> 想完全对齐就把 `joint_offset[0]` 改成 2.154534，不改也满足 < 1 mm 的验收线。

同一批测量同时确认**杆长标称值全部正确**（base 系平面投影，q=0 与 ±0.4 都不变）：

```
O-A 0.0500   A-B 0.1050(左)/0.1050(右)   D-C 0.0500   O-D 0.0600
闭链后 |C-B|：左 0.1052 / 右 0.1050（标称 l3=0.105）
```

标定后校验（`keyframe` 站姿，控制器 FK 与模型对拍）：

| 腿 | q | α / β | 模型足端 (x,y) | FK 复算 | 残差 |
|---|---|---|---|---|---|
| 左 | 0 / 0 | 123.446°(URDF) / 34.312° | (0.0455, 0.1171) | (0.0455, 0.1171) | 0.0 mm |
| 右 | −0.2259 / +0.2301 | 132.814° / 42.434° | (0.0333, 0.1173) | (0.0333, 0.1173) | 0.1 mm |

（左腿这行按 URDF 精确角 123.446° 复算，残差为 0；若仍用 yaml 的 123.499°，残差 0.05 mm。）

**不标定的后果**（改之前）：`q=0` 被当成 α=β=0 → 左腿估出 `foot_y=0.101`（真值 0.117）、
右腿 `0.094`（真值 0.117），质心高度被低估约 18 mm，VMC 于是命令腿继续伸，顶到 `[-2,2]`
限位后「贴限位 + 慢慢滑走」—— 这才是 12.4 里那个现象的**根因**，改 `com_height_default` 只是掩盖。
不要靠改 MJCF 去凑零位：换 CAD / 换减速器零位后重测这 8 个数即可。

### 12.8 修不掉的部分：无控制倒地仍然会被弹飞

**结论：这是 MuJoCo「闭链 + 接触冲击」的固有行为，不是本 MJCF 写错了。**

实测（无任何控制、闭链全开、自由塌落）：

| 场景 | 速度峰值 | 结果 |
|---|---|---|
| 闭链开、接触开 | 40～140 m/s（随参数抖动） | 机身飞出 8 m |
| 闭链关、接触开 | 11 m/s | 正常躺倒 |

已经系统性排除的参数（都没有用，抖动幅度只是随机变化）：

```
solref 0.006 / 0.01 / 0.02 / 0.05 / 0.1      solimp 0.95/0.99/0.001 ～ 0.8/0.9/0.05
solver iterations 100 / 500                  dt 0.002 / 0.001 / 0.0005
Euler / RK4 / implicitfast / implicit        armature 0 / 1e-4 / 1e-3 / 1e-2 / 1e-1
condim 1 / 3 / 4，friction 1.2 → 0.01        去掉 base_col、球替代轮碰撞体
```

只有把闭环放到极软（`solref=0.1`，残差 7 cm）才不炸，但这个精度对 VMC 完全不可用。

对照参考工程 `/home/zhangjiayi/RL/ATWheelLeg/MujocoSim/scr/car/model/scene.xml`：
同一个测试它**第 2 步就发散**（vpeak 43.7），说明这不是本模型的特性。

**所以正确的策略不是"把倒地调稳定"，而是不让它在无控制状态下倒地**：
启动门控（12.2）+ 控制器。需要观察塌落过程时，用挂架模式（`start_gate:=true` 且不使能）看前半段即可。

### 12.9 离线排查工具（不起 ROS，10 行搞定）

怀疑模型不稳定时，先用一个独立小程序把"闭链本身"和"接触"分开，比反复 launch 快得多：

```bash
g++ -O2 -o probe probe.cpp \
  -I$HOME/wheel_leg/install/mujoco_ros2_control/include \
  -L$HOME/wheel_leg/install/mujoco_ros2_control/lib -lmujoco \
  -Wl,-rpath,$HOME/wheel_leg/install/mujoco_ros2_control/lib
```

```cpp
char err[2000];
mjModel *m = mj_loadXML("src/model/model/scene.xml", nullptr, err, sizeof(err));
mjData  *d = mj_makeData(m);
mj_forward(m, d);

// 1) 闭环残差（改完 site 必须查）
for (int i = 0; i < d->nefc; i++)
  if (d->efc_type[i] == mjCNSTR_EQUALITY) printf("eq pos %.3e\n", d->efc_pos[i]);

// 2) 分离变量：依次关 equality / contact，看是谁在炸
m->opt.disableflags |= mjDSBL_EQUALITY;   // 或 mjDSBL_CONTACT

// 3) 跑一段，用速度峰值判断，不要用"超过 25 就 break"（会误判）
for (int s = 0; s < 2000; s++) mj_step(m, d);
```

经验教训：**判据要用"是否 NaN / 速度峰值多大"，不要用固定阈值提前 break**，
否则会把"正常撞击的 25 m/s 尖峰"误判成发散，然后去调一堆没用的参数（这次就浪费了很久）。

### 12.10 改动后的回归清单

1. `qpos=0` 闭环残差 < 1e-6 m（12.3）
2. `foot_y = com_height − com_offset` < 0.90 × 可达上限（12.4）；当前 `com_height_max=0.100`
   → `foot_y=0.1338`，正是 90% 这条线，节点启动的 11 点采样校验能过
3. 附录 C 的复算脚本输出与 `leg_control_params.yaml` 的 6 个模型参数逐位一致
4. `ros2 launch ... show_gui:=false` 日志顺序必须是：
   `启动门控已启用` → `控制器已全部 active` → `状态 RUN，故障 0x00` → `启动门控放开`
5. 门控期间 `ros2 topic echo /joint_states --once`：关节速度量级 1e-3，轮速不增长
6. 释放后 10 s：无 `FAULT`，轮速不发散（> 100 rad/s 就是 12.5 的 armature 没给对）
7. **站立质量**：释放后 30 s 内，`/leg_control/state` 的 `u`（下标 6，第 7 个数）应在 0 附近，饱和占比 < 10%；
   `x` 摆动 < 0.05 m、`θ` < 0.03 rad。贴边 > 50% 说明增益相对 `u_max` 太大（6.3）
8. **扰动恢复**：用 `ros2 param set /leg_control theta_offset 0.10`（0.05 s 后改回 0）模拟一次姿态冲击，
   日志应出现 `FAULT 0x08 → RUN`（约 20 ms），机器人仍站着；如果直接倒地，说明冲击已经超过倒伏判据
9. **环境干净**：`ros2 node list` 里不应出现 `ros_gz_bridge` / `backend_node` / `zlio`（同 domain 会污染数据，见 5.6）

### 12.11 站稳的调参实录（2026-10-30）

目标：`enable:=true`、`show_gui:=false` 冷启动后能长时间站着不动。

| 阶段 | 现象 | 结论 / 动作 |
|---|---|---|
| 1 | 故障位 `0x22 ↔ 0x00` 每秒抖、状态永远 `INIT`、偶尔飞出去 z=32 m / x=−124 m | 同 domain 有另一套机器人栈在发 `/joint_states`、`/clock` → 隔离 `ROS_DOMAIN_ID=42`（5.6） |
| 2 | 能站但前后摆 ±0.3 m，轮速 ±18 rad/s，`u` 常年贴 ±0.4（饱和 86%） | `K≈27` 对 `u_max=0.4` 大一个量级 → 整体缩到 **1/6**（6.3） |
| 3 | 1/6 后饱和 0%，x∈[0.020,0.030]、θ≈−0.006、z=0.106 | 稳态达标 |
| 4 | 但启动瞬态 x 一度 0.44 m | 门控放开时刻在 1.12～1.60 s 浮动、轮子按固定延时先转 → `wheel_arm_on_gate: true`（12.2） |
| 5 | 推一下报 `FAULT 0x08` 后关节失力，0.1 s 内直接倒 | 姿态类 FAULT 期间继续保平衡 + 自动回 `RUN`（11.3） |

最终一组（已在 `leg_control_params.yaml` 里）：

| 参数 | 值 |
|---|---|
| `gain_mode` / `k_coeffs` | quadratic / 常数项 `[−1.18, −4.52, −1.16, −0.28]`（LQR 的 1/6） |
| `u_max` / `balance_leak_rate` | 0.40 N·m / 60 s⁻¹ |
| `max_pitch` / `max_pitch_rate` | 0.35 rad（20°）/ 3.0 rad/s |
| 轮子使能时刻 | `wheel_arm_on_gate: true` |

验证数据：

- **冷启动**：旧阈值版连续 114 s；最终参数（0.35/3.0 + 门控同步）下 40 s、52 s 两次复测指标一致 ——
  `z=0.106 m` 恒定，`x∈[0.020,0.030] m`，`θ∈[−0.008,−0.005] rad`，`θ̇≈0`，`u≈±0.01 N·m`，
  饱和 0%，`FAULT` 计数 0，控制环无超时告警；
- **中等扰动**（`theta_offset` 阶跃 0.10 rad 保持 50 ms）：`FAULT 0x08 → RUN` 用时 22 ms，
  之后 30 s 继续站；
- **剧烈扰动**（0.20 rad / 100 ms）：会恢复 1～2 次，但瞬态角速度超过 9 rad/s 被判倒伏，仍会倒。

一个容易困惑的数：站稳时 `/leg_control/state[8]` 的 `h` 是 **0.069 m** 而不是 0.0834 m。
这是 VMC 的静态压缩：`kp_y=600 N/m`、单腿承重约 9.5 N → 压 16 mm，`foot_y` 从 0.1172 落到约 0.101，
再减 `com_offset` 就是 0.069。属于预期行为；`quadratic` 模式下增益与 `h` 无关，不影响控制。

---

## 附录 A：核心公式

**拉格朗日建模**（`q = [x, θ]`，状态 `x = [x, θ, ẋ, θ̇]`）

```
D(θ) = [ M_eff+m          m·h·cosθ ]      h(q,q̇) = [ -m·h·θ̇²·sinθ ]
       [ m·h·cosθ        I + m·h² ]               [ -m·g·h·sinθ  ]

D(θ)·q̈ + h(q,q̇) = Bq·u          M_eff = M + Jw/r²
Bq = [1, 0]ᵀ（力输入） 或 [1/r, 0]ᵀ（轮轴力矩输入）
```

**小角度线性化**（θ≈0，`det = (M_eff+m)·I + M_eff·m·h²`）

```
A = ⎡ 0        0          1  0 ⎤        B = D₀⁻¹·Bq
    ⎢ 0        0          0  1 ⎥
    ⎢ 0  −m²·g·h²/det     0  0 ⎥
    ⎣ 0  (M_eff+m)·m·g·h/det 0  0 ⎦
```

**LQR**：`u = −K(x − x_ref)`，`K = R⁻¹BᵀP`，P 由 CARE 求得。

**高度调度**：`h = foot_y(α, β) + com_offset`（五杆正运动学，见 9.1；串联 2R 的
`l1·cos(q_h) + l2·cos(q_h+q_k) + c` 只剩 `LegGeometry` 在用），再按 `K_j(h) = c2·h² + c1·h + c0` 拟合。

---

## 附录 B：下位机协议

```c
// 上位机 -> 下位机（96 B）
typedef struct {
    Wheel_t wheel[2];        // {angle, omega}         16 B
    JointTarget_t joint[4];  // {torque, angle, omega, kp, kd}  80 B
} RobotTarget_t;

// 下位机 -> 上位机（208 B，10 ms 周期）
typedef struct {
    Wheel_t wheel[2];        //                         16 B
    JointState_t joint[4];   // 7×float + uint64 error + uint8 feedback  48 B each
} RobotState_t;
```

无包头/类型/使能字段，**长度即帧**；上位机用 `FrameAssembler` 按 208 B 切分。

**固件侧行为（`stm32/one/task/task.c`）**

| 项 | 现状 |
|---|---|
| `RobotTarget_t` 接收 | `usb_trans.c` 收到短包即视为一帧完成，`usb_target_received()` 直接 `memcpy(&rob_target, ...)`，要求整帧一次到齐（96 B） |
| 轮子执行 | `wheel_control_task`：`angle` 进位置环 → 输出作为速度环目标 → 电流 int16 打包成 0x200 帧（0x201/0x202 反馈） |
| 关节执行 | `joint_control_task`：`RobStrideMotionControl(τ, θ, ω, kp, kd)`，`θ_motor = inv·θ_joint + pos_offset` |
| 状态上报 | `state_send_task` 每 10 ms 发 208 B；USB 忙则重试 3 次，仍失败则丢帧（`cdc_state_drop_cnt`） |
| 看门狗 | **无**。USB 断开后 `rob_target` 保持最后一帧 |

**频率对应关系**

| 方向 | 上位机 | 下位机 |
|---|---|---|
| 命令下行 | `usb_drive` 200 Hz | `target_receive_task` 收到即覆盖 |
| 状态上行 | `usb_drive` 按 208 B 切帧 | `state_send_task` 100 Hz |
| 电机环 | — | 333 Hz（`vTaskDelayUntil` 3 ms） |

> 上层 200 Hz 下发、下层 333 Hz 执行、状态 100 Hz 上报：三者频率不同是正常的，
> 关键约定是“下发的是**目标**而不是增量”，丢一两帧不影响收敛。

**上位机侧约定**

- `odom` **不由** `usb_drive` 发布：`leg_control_node` 用 `joint_states` 里的轮端角度差积分出前进弧长、
  再用 IMU yaw 分解成 2D 位姿，并发布 `odom→base_link` 的 TF；轮半径只在 `leg_control_params.yaml` 出现一次。
- `usb_drive` 只负责“把最近一帧命令搬到 USB”，任何超时都改发安全帧（见第 11 章）。

---

## 附录 C：URDF / MJCF 约定

**关节表**（URDF 的 `<ros2_control>`、MJCF 的 `<joint>`、仿真控制器三处名字必须完全一致）

一条腿 = 五杆闭链：2 个驱动曲柄（`*_joint1`）+ 2 个被动铰（`*_joint2`）+ 1 个轮子。
`*_joint2` **没有**进 `<ros2_control>`，在 MuJoCo 里由 `connect` 约束解出。

| 关节 | 类型 | URDF 限位 | `<ros2_control>` | MJCF actuator | 说明 |
|---|---|---|---|---|---|
| `lf_joint1` / `lb_joint1` | revolute | ±2.0 rad（effort 12 N·m） | effort | `motor` ±12 N·m | 左前/左后曲柄（α/β），髋轴 z=0.1493，x=+0.029206 / −0.030794 |
| `rf_joint1` / `rb_joint1` | revolute | ±2.0 rad（effort 12 N·m） | effort | `motor` ±12 N·m | 右腿，`rpy=+1.5708` 整体绕 x 转 180° → 关节轴在 base 系反向（见 12.7） |
| `lf_joint2` / `lb_joint2` / `rf_joint2` / `rb_joint2` | revolute | ±3.1416 rad（effort 1 N·m） | — | — | 被动膝/摇杆铰，不接收命令 |
| `lefoot_joint` / `rifoot_joint` | continuous | velocity 40 rad/s（effort 3 N·m） | effort | `motor` ±3 N·m | 左右轮，轴沿连杆 z（base 的 ±y） |

**连杆与质量**（URDF `<inertial>`，与 MJCF `<body><inertial>` 同值）

| URDF link / MJCF body | 质量 [kg] | 说明 |
|---|---|---|
| `base_link`（MJCF 带 `freejoint`） | 1.667452 | 质心 (−0.0008, 0.00002, 0.1213)，取网格包围盒中心（导出器原始值 z=1.0258 错轴，已改） |
| `lf_link1` / `rf_link1` | 0.022863 ×2 | 前曲柄，髋轴原点 z=0.1493 |
| `lb_link1` / `rb_link1` | 0.015741 ×2 | 后曲柄，髋轴原点 z=0.1493 |
| `lf_link2` / `rf_link2` | 0.035533 ×2 | 前连杆 + 轮叉（前肘→轮轴 0.105 m） |
| `lb_link2` / `rb_link2` | 0.044132 ×2 | 后摇杆（后肘→闭链点 0.105 m） |
| `lefoot_link` / `rifoot_link` | 0.130114 / 0.155050 | 轮 + 轮毂电机，轮轴 y=±0.1525（轮距 0.305），轮胎外缘 R=0.030 |

合计 2.189 kg = 摆杆 1.904 kg（`pendulum_mass`）+ 两轮 0.285 kg（`cart_mass`），与 6.1 的表一致。

**零位、home 与高度公式**

- `base_link` 原点在两轮接地点中点、z=0 即地面；但按轮胎实际半径 0.030，轮轴落在 z=0.0321，
  即原点比真实地面高约 2 mm（导出器的固定平移留的余量）。
- **URDF 关节全零不是对称站姿**：全零时左轮轴 z=0.0322、右轮轴 z=0.0186（差 13.6 mm，MJCF 里
  实测 gap = 0.0135 m）。
  真正的 home 由 MJCF `keyframe "home"` 给出：左腿 `q=0`，右腿
  `rf1=-0.2259 / rf2=+0.3339 / rb1=+0.2301 / rb2=-0.3302`，两轮零速同时着地（残差 0.15 mm）。
- 坐标系：MJCF 用 CAD 约定（x 前 / y 下 / z 左），`base_link` 的 `pos="0 0 -0.8712"`；
  URDF 已换成 REP-103（x 前 / y 左 / z 上），两者差一个绕 x 的 −90° 旋转，`sim_bridge` 的
  IMU 姿态换算按 MJCF 写。所有 hinge 轴互相平行（平面机构），MJCF 靠两条
  `connect`（`lb_rocker_axle↔lefoot_axle`、`rb_rocker_axle↔rifoot_axle`）闭合。
- 五杆几何（两条腿共用）：`l1=0.050`（O→A）、`l2=0.105`（A→轮轴）、`l3=0.105`（C→B）、
  `l4=0.050`（D→C）、`l5=0.060`（O→D）。**LQR 的高度不再用串联 2R 公式**，改成
  “髋→足端”距离加质心偏置：

  ```
  foot_y = 髋轴 z − 轮轴 z        h = foot_y + com_offset
  ```

  home 处 `foot_y = 0.1172 m`、`com_offset = -0.0338 m` → `h = 0.0834 m`；五杆足端竖直可达上限
  约 0.148 m（曲柄与连杆共线时奇异），见 12.4。

**改模型时的自检**

改完两边跑一次，确认没有漏改的关节名：

```bash
check_urdf src/model/model/wheel_leg.urdf
python3 - <<'EOF'
import xml.etree.ElementTree as ET
u = ET.parse('src/model/model/wheel_leg.urdf').getroot()
m = ET.parse('src/model/model/wheel_leg.xml').getroot()
ctl = {j.get('name') for j in u.findall('ros2_control/joint')}
joints = {j.get('name') for j in m.iter('joint')}
acts = {a.get('joint') for a in m.find('actuator')}
print('control - mjcf :', ctl - joints or 'OK')
print('control - act  :', ctl - acts or 'OK')
EOF
```

三项都打印 `OK` 才说明 URDF、MJCF、控制器的命名闭环了。

**模型参数复算脚本**（改了 CAD / 导出新 URDF 后跑一遍，把输出对回 `leg_control_params.yaml`）：

```python
# python3 check_model.py（在仓库根目录跑，需要 numpy）
import numpy as np, xml.etree.ElementTree as ET
HOME = {'rf_joint1': -0.225943, 'rf_joint2': 0.333931,
        'rb_joint1': 0.230087, 'rb_joint2': -0.330171}    # MJCF keyframe "home"
WHEELS = ('lefoot_link', 'rifoot_link')
u = ET.parse('src/model/model/wheel_leg.urdf').getroot()
L = {l.get('name'): l for l in u.findall('link')}
J = {j.get('name'): j for j in u.findall('joint')}

def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
            @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))

def axang(a, q):
    a = a / np.linalg.norm(a); K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K

def org(e):
    o = e.find('origin')
    if o is None: return np.zeros(3), np.eye(3)
    return (np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()]),
            rpy2R(*[float(v) for v in (o.get('rpy') or '0 0 0').split()]))

def fk(link='base_link', R=np.eye(3), p=np.zeros(3), out=None):
    out = {} if out is None else out
    out[link] = (R, p)
    for j in J.values():
        if j.find('parent').get('link') != link: continue
        xyz, Rj = org(j); Rn, pn = R @ Rj, p + R @ xyz
        if j.get('type') in ('revolute', 'continuous'):
            Rn = Rn @ axang(np.array([float(v) for v in j.find('axis').get('xyz').split()]),
                            HOME.get(j.get('name'), 0.0))
        fk(j.find('child').get('link'), Rn, pn, out)
    return out

def inertial(link):
    i = L[link].find('inertial'); xyz, Rj = org(i); m = float(i.find('mass').get('value'))
    t = i.find('inertia')
    I = np.array([[float(t.get('ixx')), float(t.get('ixy')), float(t.get('ixz'))],
                  [float(t.get('ixy')), float(t.get('iyy')), float(t.get('iyz'))],
                  [float(t.get('ixz')), float(t.get('iyz')), float(t.get('izz'))]])
    return xyz, Rj, m, I

pose = fk(); body = [l for l in L if l not in WHEELS]
m_cart = sum(inertial(w)[2] for w in WHEELS)
jw = sum(inertial(w)[3][2, 2] for w in WHEELS)
m_pend = sum(inertial(l)[2] for l in body)
c = sum(inertial(l)[2] * (pose[l][1] + pose[l][0] @ inertial(l)[0]) for l in body) / m_pend
I = 0.0
for l in body:
    xyz, Rj, m, Im = inertial(l); R, p = pose[l]; d = p + R @ xyz - c
    I += (R @ Rj @ Im @ Rj.T @ R.T)[1, 1] + m * (d[0] ** 2 + d[2] ** 2)
hip_z = np.mean([pose[l][1][2] for l in ('lf_link1', 'lb_link1', 'rf_link1', 'rb_link1')])
axle_z = np.mean([pose[w][1][2] for w in WHEELS])
sep = abs(pose['lefoot_link'][1][1] - pose['rifoot_link'][1][1])
print(f'cart_mass {m_cart:.3f}  pendulum_mass {m_pend:.3f}')
print(f'com_length {c[2]-axle_z:.4f}  com_inertia {I:.5f}  wheel_inertia {jw:.3g}')
print(f'com_offset {c[2]-hip_z:+.4f}  wheel_separation {sep:.3f}  foot_y(home) {hip_z-axle_z:.4f}')
```

当前 URDF 的输出（应与 yaml 完全一致）：

```
cart_mass 0.285  pendulum_mass 1.904
com_length 0.0834  com_inertia 0.00255  wheel_inertia 5.32e-05
com_offset -0.0338  wheel_separation 0.305  foot_y(home) 0.1172
```
