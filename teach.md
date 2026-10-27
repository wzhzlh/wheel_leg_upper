# 双并腿轮足机器人（wheel_leg）工程说明

本工程实现一条完整的自平衡控制链路：**IMU 姿态 → 欧拉角 → LQR 状态反馈 → USB-CDC → 下位机电机**，
配套拉格朗日动力学建模、小角度线性化、LQR 求解与 **增益随质心高度调度 K(h)**；
另有一套 **五杆腿的足端 VMC**（虚拟弹簧-阻尼 + 雅可比力矩映射，第 9 章）和一套
**差速底盘运动学**（`cmd_vel` → 左右轮目标转速，第 10 章），
以及一套 **MuJoCo 仿真模型**（无 STL 的 URDF + MJCF），可以完全不接硬件就把整机跑起来。

> 阅读顺序建议：第 1~2 章看全局 → 第 3 章查文件 → 第 5 章跑起来 → 第 6 章调参 → 第 9/10 章看 VMC 与差速。
> 只想看仿真：直接跳到 **5.6 MuJoCo 仿真**，附录 C 是 URDF/MJCF 的约定表。

---

## 1. 系统架构与数据流

```
              ┌──────────────────┐
   USB 串口   │ imu_ros2_device  │  /imu/data (sensor_msgs/Imu: 四元数/角速度/加速度)
   YbImu  ───►│ ybimu_driver     │──────────────┐
              └──────────────────┘              │
                                                ▼
  USB-CDC     ┌──────────────────┐      ┌────────────────────────┐
  下位机  ───►│ usb_drive_node   │      │ leg_calc_lqr_controller│
  (STM32)     │  208B 状态帧     │─/odom→│   θ = pitch            │
              │                  │─/joint_states→  h = h(4 joints)│
              │  96B 目标帧      │      │   K = K(h)             │
              └──────────────────┘      │   u = -K(x - x_ref)    │
                     ▲                  └───────────┬────────────┘
                     │  /leg_calc/control (u)       │
                     └──────────────────────────────┘

  腿（力矩通道，可选）：
  /joint_states ──► leg_calc_vmc_controller ──► /joint_target（4 关节力矩）
                    五杆 FK/J → 足端弹簧阻尼 → τ = JᵀF

  行驶（轮速通道，可选）：
  /cmd_vel ──► leg_calc_diff_drive ──► /wheel_target（[左, 右] rad/s）
               ω_L=(v−L/2·ω)/r, ω_R=(v+L/2·ω)/r
             与 LQR 的 u×gain 相加后由 usb_drive 下发 → 边走边平衡
``

| 环节 | 包 | 关键话题 |
|---|---|---|
| IMU 采集 | `imu_ros2_device` | 发布 `/imu/data` |
| 状态估计 + 轮子 LQR | `leg_calc` | 订阅 `/imu/data` `/odom` `/joint_states`，发布 `/leg_calc/control` 等 |
| 足端 VMC（腿） | `leg_calc` | 订阅 `/joint_states`，发布 `/joint_target`（关节力矩） |
| 差速行驶（轮） | `leg_calc` | 订阅 `/cmd_vel`，发布 `/wheel_target`（左右轮目标转速） |
| 下位机通信 | `usb_drive` | 订阅 `/leg_calc/control` `/joint_target`，发布 `/odom` `/joint_states` |
| 总启动 | `launch_pack` | 一条命令拉起以上三个包 |
| 模型描述 | `model` | URDF + MJCF + 仿真控制器配置 |
| 物理仿真 | `mujoco_ros2_control` | 用 MJCF 代替真实硬件，提供同一套 `ros2_control` 接口 |

**仿真链路**（与实机并列，二者共用 `ros2_control` 关节名）：

```
   model/model/wheel_leg.urdf ──► robot_state_publisher ──► /robot_description + TF
                                          │
   model/model/scene.xml ─────► mujoco_ros2_control ──► controller_manager
   (含 wheel_leg.xml: freejoint/                │
    actuator/sensor)                            ├─► joint_state_broadcaster  /joint_states
                                                ├─► leg_position_controller  /leg_position_controller/commands
                                                └─► wheel_effort_controller  /wheel_effort_controller/commands
```

实机是 `usb_drive` 下发 96 B 目标帧，仿真里换成 `ros2_control` 控制器直接写 MuJoCo 的 `ctrl`；
关节名、连杆名、质量、尺寸完全一致，所以算法侧代码不需要为仿真改任何一行。

---

## 2. 目录结构

```
/home/zhangjiayi/wheel_leg/
├── teach.md                       ← 本文档
├── build/  install/  log/         ← colcon 生成，勿手改
└── src/
    ├── launch_pack/               ← 唯一的启动包（实机 + 仿真）
    │   ├── launch/wheel_leg_bringup.launch.py      ← 实机总启动
    │   ├── launch/wheel_leg_mujoco_sim.launch.py   ← MuJoCo 仿真启动
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── imu_ros2_device/           ← YbImu 串口 IMU 驱动（ament_python）
    │   ├── imu_ros2_device/ybimu_driver.py
    │   ├── setup.py  package.xml  resource/
    │
    ├── leg_calc/                  ← 动力学建模 + LQR + 足端 VMC（ament_cmake, C++）
    │   ├── include/leg_calc/
    │   │   ├── wheel_leg_model.hpp    ← 拉格朗日模型 + 解析线性化
    │   │   ├── lqr.hpp                ← Riccati 求解 + K(h) 调度 + 腿部几何
    │   │   ├── foot_vmc.hpp           ← 五杆正/逆运动学 + 雅可比 + 足端弹簧阻尼
    │   │   ├── diff_drive.hpp         ← 差速底盘正/反解（v,ω ↔ 左右轮速）
    │   │   ├── quat_utils.hpp         ← 四元数 → 欧拉角
    │   │   └── param_utils.hpp        ← 节点参数声明/读取
    │   ├── src/
    │   │   ├── wheel_leg_model.cpp
    │   │   ├── lqr.cpp
    │   │   ├── foot_vmc.cpp
    │   │   ├── diff_drive.cpp
    │   │   ├── lqr_controller_node.cpp ← 轮子 LQR 平衡控制器（主节点）
    │   │   ├── vmc_controller_node.cpp ← 足端 VMC：/joint_states → /joint_target
    │   │   ├── diff_drive_node.cpp     ← 差速底盘：/cmd_vel → /wheel_target
    │   │   ├── model_demo_node.cpp     ← 离线仿真演示
    │   │   └── gain_table_tool.cpp     ← K(h) 扫描 + 二次拟合
    │   ├── config/
    │   │   ├── lqr_params.yaml         ← LQR 控制器全部参数
    │   │   ├── vmc_params.yaml         ← VMC 几何 / Kp / Kd / 足端目标
    │   │   └── diff_drive_params.yaml  ← 轮距 / 轮半径 / 限幅
    │   └── test/
    │       ├── test_wheel_leg_model.cpp
    │       ├── test_foot_vmc.cpp
    │       └── test_diff_drive.cpp
    │
    ├── usb_drive/                 ← USB-CDC 通信（ament_cmake, C++ + libusb）
    │   ├── include/usb_drive/
    │   │   ├── cdc_trans.hpp          ← libusb 异步收发 + 定长帧重组
    │   │   └── data_pack.h            ← 与固件逐字节一致的协议结构体
    │   ├── src/
    │   │   ├── cdc_trans.cpp
    │   │   └── usb_drive_node.cpp     ← 通信节点
    │   └── test/test_frame_assembler.cpp
    │
    ├── model/                     ← 机器人描述 + 仿真配置（ament_cmake，纯资源包）
    │   ├── model/
    │   │   ├── wheel_leg.urdf          ← 无 STL 的整机 URDF（含 ros2_control 标签）
    │   │   ├── wheel_leg.xml           ← 与 URDF 一一对应的 MJCF
    │   │   └── scene.xml               ← 地面 + 光照，include wheel_leg.xml
    │   ├── config/wheel_leg_controllers.yaml
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── mujoco_ros2_control/       ← 第三方 MuJoCo 硬件插件（FetchContent 拉 MuJoCo 源码）
        ├── include/mujoco_ros2_control/mujoco_system.hpp
        ├── src/mujoco_system.cpp       ← ros2_control 硬件接口实现（读写 qpos/qvel/ctrl）
        ├── src/mujoco_ros2_control_plugin.cpp
        ├── src/*_parameters.yaml       ← generate_parameter_library 的参数定义
        └── ...
```

下位机固件（不在本仓库）：`/home/zhangjiayi/qq_down/stm32/one/`

| 文件 | 作用 |
|---|---|
| `task/task.h` | 上下位机协议结构体（`RobotState_t` / `RobotTarget_t`）、`JointCalib_t`、任务声明 |
| `task/task.c` | 4 个任务：轮子 PID、关节 RobStride 控制、状态上报、目标接收；CAN 接收回调 |
| `Core/Src/usb_trans.c` | CDC 接收组包（收到短包即视为一帧结束） |
| `Core/Inc/RobStride2.h` / `Core/Src/RobStride2.c` | RobStride 电机运动控制与反馈解析 |

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
| `wheel_leg_model.hpp/.cpp` | **核心模型**。`ModelParams`（M, m, h, I, g, r, Jw, `input_mode`）；`massMatrix(θ)`/`biasVector(θ,θ̇)`/`inputMatrix()` 组成 `D(q)q̈ + h(q,q̇) = Bq·u`；`dynamics()` 给出 `ẋ`；`linearize()` 给出解析 A、B；`linearizeNumeric()` 数值雅可比（用于校验）；`totalEnergy()`。头文件里写有完整拉格朗日推导。 |
| `lqr.hpp/.cpp` | **LQR 全链路（原来 3 个文件合并而来）**。`solveContinuousLqr()`：解 `AᵀP + PA − PBR⁻¹BᵀP + Q = 0`，哈密顿矩阵 → 特征分解取稳定子空间 → `P = V2V1⁻¹` → `K = R⁻¹BᵀP`，返回残差、闭环极点、`isStable()`。`LegGeometry`：由 4 个关节角算质心高度 `h = l1·cos(q_hip) + l2·cos(q_hip+q_knee) + com_offset`（双腿取平均）。`GainSchedule`：`kQuadratic` 用二次多项式 `K_j(h)=c2h²+c1h+c0`；`kOnlineLqr` 按 h 重建模型现解一次（带 h 缓存）。系数全零自动退回在线 LQR。 |
| `quat_utils.hpp` | `quaternionToRpy()`：四元数 → (roll, pitch, yaw)，ZYX 顺序，内部归一化。 |
| `param_utils.hpp` | 节点共用的参数层：`declareModelParams()` / `readModelParams()`（7 个物理参数 + `input_mode`）、`declareLqrWeights()` / `readLqrWeights()`（`q_diag` / `r_weight` → Q/R）。支持传入 pending 参数，用于 `ros2 param set` 回调。 |
| `foot_vmc.hpp/.cpp` | **足端 VMC 核心**（详见第 9 章）。`FiveBarGeometry`：五杆正运动学 `T(q)`、解析雅可比 `J(q)=∂x/∂q`、逆运动学 `T⁻¹(x)`（可选种子选枝）、逆雅可比、`τ=JᵀF`。`SpringDamperParams` + `springDamperForce()`：逐轴弹簧-阻尼 + 力限幅。`computeVmcLeg()`：一条腿的 FK → 足端速度 → PD → 力矩全链路。 |
| `diff_drive.hpp/.cpp` | **差速底盘运动学**（详见第 10 章）。`bodyFromWheels()`：`v=(ω_L+ω_R)r/2`、`ω=(ω_R−ω_L)r/L`；`wheelsFromBody()`：`ω_L=(v−L/2·ω)/r`、`ω_R=(v+L/2·ω)/r`，先限 v/ω，再按同一比例缩放轮速（保持转向半径）。 |
| `lqr_controller_node.cpp` | **轮子平衡控制器（主节点）**。订阅 IMU/里程计/关节状态，组装 `x=[x,θ,ẋ,θ̇]`，按当前 h 取 K，算 `u=-K(x-x_ref)` 并限幅。参数支持运行时 `ros2 param set` 热更新。 |
| `vmc_controller_node.cpp` | **足端 VMC 节点**。订阅 `/joint_states`，两腿各算一次 VMC，把关节力矩发到 `/joint_target`（`effort` 字段），并发布 `vmc/state` 调试量。关节超时/无解/奇异时力矩置零。 |
| `diff_drive_node.cpp` | **差速底盘节点**。订阅 `/cmd_vel`（`linear.x`、`angular.z`），50 Hz 发布 `/wheel_target`（`[左轮, 右轮]` rad/s），`diff_drive/state` 给调试量；`cmd_vel` 超时后发一帧零速。 |
| `model_demo_node.cpp` | 离线演示：读参数 → 打印 A/B/K/闭环极点 → 用**非线性模型** RK4 闭环仿真，验证线性控制器在原系统上有效。不接硬件即可跑。 |
| `gain_table_tool.cpp` | 扫 h → 在线 LQR → 最小二乘二次拟合，直接打印可粘贴的 `k_coeffs`。 |
| `config/lqr_params.yaml` | 轮子 LQR 控制器全部参数（见 6.9 速查表）。 |
| `config/vmc_params.yaml` | 足端 VMC 参数：五杆长度、装配分支、Kp/Kd、足端目标、限幅（见 9.3）。 |
| `config/diff_drive_params.yaml` | 差速底盘参数：轮距、轮半径、v/ω/轮速限幅、指令超时（见 10.2）。 |
| `test/test_wheel_leg_model.cpp` | 11 个单测：解析 vs 数值线性化、A 阵手推校核、质量矩阵正定、能量守恒、Riccati 残差、非线性闭环收敛、四元数转换、h 计算、K(h) 多项式、在线 LQR 一致性、系数全零回退。 |
| `test/test_foot_vmc.cpp` | 15 个单测：五杆几何合法性、中位足端位置手算校核、不可装配检测、雅可比 vs 数值差分、逆解往返（固定分支 + 种子连续跟踪）、逆雅可比往返、虚功一致性 `F·ẋ = τ·q̇`、PD 零误差/回正/阻尼/限幅、整链与奇异位形。 |
| `test/test_diff_drive.cpp` | 10 个单测：直行/原地左转/右转的正解符号、反解、正反解往返、轮速限幅保持转向半径、车体量限幅、`<=0` 表示不限、非法参数返回零。 |

### 3.3 usb_drive

| 文件 | 作用 |
|---|---|
| `cdc_trans.hpp/.cpp` | **传输层（收发 + 拆帧）**。`CdcTrans`：libusb 封装，按 VID/PID 打开、卸载 `cdc_acm`、抢占接口、异步 bulk 接收（循环重提交）、同步 bulk 发送、热插拔注册、掉线 500 ms 自动重连；`close()` 会先取消传输并等回调结束再释放，避免 use-after-free。`FrameAssembler`：定长帧重组，USB bulk 会粘包/拆包，按 208 B 累积切帧；`reset()` 可在回调内安全调用（失步重同步）。 |
| `data_pack.h` | 与固件逐字节一致的协议：`Wheel_t`(8B)、`JointTarget_t`(20B)、`JointState_t`(48B)、`RobotTarget_t`(**96B**)、`RobotState_t`(**208B**)。带 `static_assert` 卡死长度与字段偏移。 |
| `usb_drive_node.cpp` | 通信节点：下行把 **平衡分量 `u` + 行驶分量 `wheel_target`** 相加成左右轮目标（速度模式）并打包 4 关节目标；上行解析状态帧，发布 `odom`(x, ẋ)、`joint_states`(2 轮 + 4 关节)、`usb_drive/state`(原始量)。带看门狗：两路指令都超时→轮子位置保持、关节失力。`left/right_invert` 在上下行**同时**生效。 |
| `test/test_frame_assembler.cpp` | 5 个单测：协议布局、粘包、拆包、残帧、回调内 reset。 |

### 3.4 launch_pack（唯一启动包）

| 文件 | 作用 |
|---|---|
| `launch/wheel_leg_bringup.launch.py` | 实机总启动：`ybimu_driver` + `lqr_controller_node` + `usb_drive_node`（可选 `vmc_controller_node` / `diff_drive_node`），带 `use_imu` / `use_lqr` / `use_vmc` / `use_diff_drive` / `enable_motor` 开关；参数 `port` / `imu_frame_id` / `imu_rate` / `params_file` / `vmc_params_file` / `diff_drive_params_file` / `vid` / `pid` / `send_rate` / `wheel_cmd_mode`。IMU 话题由驱动的 `topic` 默认值与 `lqr_params.yaml` 的 `imu_topic` 共同决定，两边需一致。 |
| `launch/wheel_leg_mujoco_sim.launch.py` | 仿真总启动：`robot_state_publisher` + `mujoco_ros2_control`，用 `OnProcessStart` 在 MuJoCo 起来后 spawn 三个控制器。参数：`mjcf_path` / `show_gui` / `show_rviz` / `simulation_frequency` / `real_time_factor`。资源路径通过 `FindPackageShare('model')` 取得。 |

> 各包不再自带 launch：单跑某一部分时用 bringup 的开关，例如
> `ros2 launch launch_pack wheel_leg_bringup.launch.py enable_motor:=false`（只跑 IMU + LQR）。

### 3.5 model（仿真模型与配置）

全部是资源文件，没有编译产物，`CMakeLists.txt` 只做 `install(DIRECTORY model config)`。

| 文件 | 作用 |
|---|---|
| `model/wheel_leg.urdf` | **整机 URDF，不用任何 STL**，全部是 box/cylinder。含 6 个驱动关节的 `<ros2_control>`（腿=position 命令、轮=effort 命令），`base_link` → 大腿 → 小腿 → 轮的串并联拓扑，左右髋在 y 上 ±0.10。外观按实机：橙色片状腿连杆 + 黑色机身 + 顶部黑色防护架 + 黑色轮（胎 + 浅灰毂）。 |
| `model/wheel_leg.xml` | 与 URDF 一一对应的 **MJCF**。`base_link` 带 `<freejoint>`（浮动基座），腿关节用 `position` actuator、轮用 `motor` actuator，`<sensor>` 里挂 `gyro`/`accelerometer`（`imu_site`）与 `framepos`/`framequat`（`base_link`）。防护架和轮毂设 `contype=0` 纯视觉，与 URDF 的 collision 一致。 |
| `model/scene.xml` | MuJoCo 场景：地面（checker 材质）+ 光照 + `include wheel_leg.xml`。`mjcf_path` 指向本文件而不是 `wheel_leg.xml`，否则仿真里没有地面。 |
| `config/wheel_leg_controllers.yaml` | 三个控制器：`joint_state_broadcaster`、`leg_position_controller`（4 腿）、`wheel_effort_controller`（2 轮），`update_rate: 500`。 |

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

## 4. 话题一览

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU → 控制器 | 四元数姿态 + 角速度 + 加速度 |
| `/odom` | `nav_msgs/Odometry` | 下位机 → 控制器 | `pose.position.x`、`twist.linear.x` 由轮角折算 |
| `/joint_states` | `sensor_msgs/JointState` | 下位机 → 控制器 | `wheel_left/right_joint` + 4 个腿关节 |
| `/joint_target` | `sensor_msgs/JointState` | 上位机 → 通信节点 | 4 个腿关节的 `position/velocity/effort`；`effort` 由 VMC 节点写入（力矩） |
| `/vmc/state` | `Float64MultiArray` | 调试 | 每腿 8 个数：`[x, y, ẋ, ẏ, Fx, Fy, τ_α, τ_β]`（左腿在前，单位 m / m·s⁻¹ / N / N·m） |
| `/cmd_vel` | `geometry_msgs/Twist` | 用户 → 差速底盘 | `linear.x` = v [m/s]，`angular.z` = ω [rad/s]（ω>0 左转） |
| `/wheel_target` | `Float64MultiArray` | 差速底盘 → 通信节点 | `[左轮, 右轮]` 目标角速度 [rad/s]，与 LQR 的 `u` 折算量相加后下发 |
| `/diff_drive/state` | `Float64MultiArray` | 调试 | `[v, ω, v_L, v_R, ω_L, ω_R, saturated]` |
| `/leg_calc/control` | `Float64MultiArray` | 控制器 → 通信节点 | `[u]`（默认力矩 N·m） |
| `/leg_calc/state` | `Float64MultiArray` | 调试 | `[x, θ, ẋ, θ̇]` |
| `/leg_calc/gain` | `Float64MultiArray` | 调试/拟合 | 当前 4 个增益 `[k_x,k_θ,k_ẋ,k_θ̇]` |
| `/leg_calc/com_height` | `Float64MultiArray` | 调试/拟合 | 当前质心高度 h |
| `/leg_calc/euler` | `Vector3Stamped` | 调试 | roll/pitch/yaw |
| `/usb_drive/state` | `Float64MultiArray` | 调试 | 原始反馈（轮角/轮速 + 4 关节角/角速度） |

**仿真相关**（只在跑 MuJoCo 时出现）：

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 仿真 → 观察 | 由 `joint_state_broadcaster` 发布，6 个关节的 position/velocity/effort |
| `/leg_position_controller/commands` | `Float64MultiArray` | 用户 → 仿真 | 4 个腿关节目标角 `[左髋, 左膝, 右髋, 右膝]` [rad] |
| `/wheel_effort_controller/commands` | `Float64MultiArray` | 用户/算法 → 仿真 | 2 个轮子力矩 `[左轮, 右轮]` [N·m] |
| `/clock` | `rosgraph_msgs/Clock` | 仿真 → 全系统 | 仿真时间，频率 = `simulation_frequency` |
| `/base_link_pose_sensor/pose` | `geometry_msgs/PoseStamped` | 仿真 → 观察 | 机身世界位姿（来自 `framepos`+`framequat`） |
| `/imu_site_imu_sensor/imu` | `sensor_msgs/Imu` | 仿真 → 观察 | 只有角速度和线加速度，**没有 orientation**（见 8.5） |

> 命令数组顺序 = URDF 里 `<ros2_control>` 的声明顺序，改 yaml 的 `joints` 列表时两边要一起改。

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

### 5.1 第一层：纯算法（不需要任何硬件）

```bash
# 看模型/线性化/LQR/闭环仿真结果
ros2 run leg_calc model_demo_node --ros-args -p sim_time:=2.0 -p dt:=0.001 \
  -p cart_mass:=1.2 -p pendulum_mass:=2.5 -p com_length:=0.28 -p com_inertia:=0.05

# 跑单元测试（11 + 5 项）
./build/leg_calc/test_wheel_leg_model
./build/usb_drive/test_frame_assembler
```

`model_demo_node` 会打印 A、B、K、闭环极点，最后打印非线性闭环仿真的收敛结果，用来确认「模型和控制器是对的」。

### 5.2 第二层：台架（有 IMU，不动电机）

```bash
ros2 launch launch_pack wheel_leg_bringup.launch.py enable_motor:=false
```

验证点：

```bash
ros2 topic hz /imu/data            # 应为 ~100 Hz
ros2 topic echo /leg_calc/euler    # 机器前后倾时 pitch 变化，单位 rad
ros2 topic echo /leg_calc/com_height
ros2 topic echo /leg_calc/gain
ros2 topic echo /leg_calc/control
```

把机器缓慢前后倾，`/leg_calc/control` 应朝相反方向增大；`/leg_calc/com_height` 应随腿的弯曲变小。

### 5.3 第三层：整机

```bash
# 先把机器吊起/架空，确认状态帧正常
ros2 launch launch_pack wheel_leg_bringup.launch.py
ros2 topic hz /odom          # 应为 ~100 Hz（下位机 10 ms 上报）
ros2 topic echo /usb_drive/state   # 前 4 个数是左右轮角/角速度，其后是 4 关节角/角速度
```

确认无异常后落地。**上电前务必确认 `theta_offset` 已标定**（见 6.3），否则一上电就会输出饱和力矩。

### 5.4 分段调试（同一个 bringup，用开关裁链路）

```bash
# 只跑 IMU 驱动
ros2 launch launch_pack wheel_leg_bringup.launch.py \
  use_lqr:=false enable_motor:=false port:=/dev/ttyUSB0

# IMU + LQR（不下发电机指令）
ros2 launch launch_pack wheel_leg_bringup.launch.py enable_motor:=false

# 只跑下位机通信（不带感知与控制）
ros2 launch launch_pack wheel_leg_bringup.launch.py \
  use_imu:=false use_lqr:=false wheel_cmd_mode:=velocity
```

### 5.5 记录与可视化

```bash
ros2 bag record /imu/data /odom /joint_states /leg_calc/state /leg_calc/control \
               /leg_calc/gain /leg_calc/com_height
# 或用 PlotJuggler 实时看曲线
ros2 run plotjuggler plotjuggler
```

### 5.6 MuJoCo 仿真（不接任何硬件）

**启动**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py                      # 带 MuJoCo 窗口
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_gui:=false      # 服务器/无显示器
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py show_rviz:=true      # 额外开 RViz（需手动加 RobotModel）
ros2 launch launch_pack wheel_leg_mujoco_sim.launch.py simulation_frequency:=1000.0 real_time_factor:=0.5
```

启动成功的标志（日志里依次出现）：

```
loaded mujoco model
[wheel_leg_system]: leg_left_hip_joint / ... / wheel_right_joint      ← 6 个关节全部匹配
[leg_left_hip_act]: added position actuator for joint: ...            ← 4 个 position
[wheel_left_act]: added effort actuator for joint: ...                ← 2 个 effort
Configured and activated joint_state_broadcaster / leg_position_controller / wheel_effort_controller
```

**手动摆姿态 / 转轮子**

```bash
# 腿关节目标角：顺序固定为 [左髋, 左膝, 右髋, 右膝]，单位 rad
ros2 topic pub --once /leg_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.25, -0.55, 0.25, -0.55]}"

# 轮子力矩（N·m），持续发才会持续给力矩
ros2 topic pub -r 100 /wheel_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2, 0.2]}"

# 看回读状态
ros2 topic echo /joint_states
ros2 topic echo /base_link_pose_sensor/pose
```

**验证仿真是否"接对了"**

1. 发 `[0.2, -0.6, 0.2, -0.6]`，`/joint_states` 里 `leg_left_hip_joint` 应到 ~0.20、`leg_left_knee_joint` 到 ~-0.60；
2. 发正力矩 `[0.2, 0.2]`，`wheel_left_joint.position` 应单调增加（轮子往前滚）；
3. `/clock` 频率应等于 `simulation_frequency`。

**几个必须知道的行为**

- 整机是**浮动基座**（MJCF 里 `base_link` 带 `freejoint`），不会自动站稳，腿一伸直就会倒下 —— 这是正常的，不是模型错了。想看它站着，需要闭环控制（见 8.5）。
- 腿关节是**位置伺服**（MJCF `position actuator`，`kp=30`），只接受目标角，不接受力矩；轮子是**力矩直通**（`motor`），与 LQR 的 `u` 同一量纲。
- 仿真的轮子力矩限幅是 ±3 N·m（MJCF `ctrlrange`），与 `lqr_params.yaml` 的 `u_max: 5.0` 不一致，跑闭环前要么改 `ctrlrange` 要么把 `u_max` 调到 3。
- `use_sim_time`：launch 没有统一打开。要让 `leg_calc` 吃仿真时间，需要给节点加 `-p use_sim_time:=true`，否则它用墙钟，和 `/clock` 会不同步。

### 5.7 足端 VMC（不需要电机也能空跑）

```bash
# 1) 不带硬件：手动喂关节角，看足端位置/力/力矩
ros2 run leg_calc vmc_controller_node --ros-args \
  --params-file install/leg_calc/share/leg_calc/config/vmc_params.yaml

# 2) 另开一个终端，假装关节在 α=β=90° 的位置（50 Hz 持续发，否则 0.2 s 后力矩归零）
ros2 topic pub -r 50 /joint_states sensor_msgs/msg/JointState \
  "{name: [leg_left_hip_joint, leg_left_knee_joint, leg_right_hip_joint, leg_right_knee_joint],
    position: [1.5708, 1.5708, 1.5708, 1.5708], velocity: [0.0, 0.0, 0.0, 0.0]}"

# 3) 看结果：足端应在 (0.050, 0.294) m；目标 0.300 时 Fy ≈ +1.9 N（往下推）
ros2 topic echo /vmc/state
ros2 topic echo /joint_target

# 4) 整机：与 LQR 一起开（轮子走 LQR，腿走 VMC）
ros2 launch launch_pack wheel_leg_bringup.launch.py use_vmc:=true
```

上电前先把 `enable_motor:=false` 跑一遍，确认 `/joint_target` 的四个力矩符号与幅值合理（吊起机器时腿应该是"撑住"而不是"往下砸"）。

### 5.8 差速行驶（不接电机也能空跑）

```bash
# 1) 只跑差速底盘
ros2 run leg_calc diff_drive_node --ros-args \
  --params-file install/leg_calc/share/leg_calc/config/diff_drive_params.yaml

# 2) 持续发指令（0.3 s 不发就自动停）
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"   # 直行
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"   # 原地左转

# 3) 看分解结果：直行 [7.69, 7.69]，左转 [-1.54, 1.54]
ros2 topic echo /wheel_target
ros2 topic echo /diff_drive/state

# 4) 整机：与 LQR 一起开（轮子 = 平衡量 + 行驶量）
ros2 launch launch_pack wheel_leg_bringup.launch.py use_diff_drive:=true
```

---

## 6. 调参手册

### 6.0 调参顺序（不要跳步）

```
① 机械标定（关节零位/方向、theta_offset）
   ↓
② 模型参数（M, m, h, I, r, Jw）+ 腿部几何（thigh/shank/com_offset）
   ↓
③ 权重 Q/R → 观察台架响应（先 use_measured_height:=false 固定 h）
   ↓
④ 打开高度调度：gain_table 拟合 k_coeffs → 填 yaml → online 对比校验
   ↓
⑤ 通信与限幅（send_rate, u_max, command_timeout）
   ↓
⑥ 整机试跑
```

### 6.1 物理参数怎么给

| 参数 | 含义 | 怎么获得 |
|---|---|---|
| `cart_mass` M | 轮子 + 底盘的等效平动质量 [kg] | 电子秤称轮组/底盘 |
| `pendulum_mass` m | 腿 + 机身质量 [kg] | 称重 |
| `com_length` h | **轮轴到质心**距离 [m] | 悬挂法找质心，或按腿伸直姿态估 |
| `com_inertia` I | 摆杆绕自身质心转动惯量 [kg·m²] | 近似 `m·L²/12`；不确定时取小值影响不大 |
| `wheel_radius` r | 轮半径 [m] | 卡尺 |
| `wheel_inertia` Jw | 两轮绕轮轴等效转动惯量 [kg·m²] | 近似 `2 × 0.5·m_w·r²` |

> 等效平动质量 `M_eff = M + Jw/r²` 由代码内部计算，不用手填。

### 6.2 θ 的零位：`theta_offset`

1. 把机器摆到「你要当作直立平衡点」的姿态并静止；
2. 读 `/leg_calc/euler` 的 `y`（pitch），记为 `θ0`；
3. 令 `theta_offset = θ0`（代码里 `θ = pitch − theta_offset`）。

运行时可直接改，不用重启：

```bash
ros2 param set /leg_calc_lqr_controller theta_offset 0.02
```

如果 IMU 装成侧向的（俯仰在 roll 上），改 `theta_axis: roll`。

### 6.3 权重 Q / R

代价函数 `J = ∫(xᵀQx + uᵀRu)dt`，`q_diag = [q_x, q_θ, q_ẋ, q_θ̇]`。

| 想要的效果 | 怎么改 |
|---|---|
| 摆杆更"硬"、倒得少 | 增大 `q_θ`（默认 500，可试 1000~5000） |
| 位置更"粘"、少漂 | 增大 `q_x`（默认 10） |
| 减少来回晃动、抑制速度 | 增大 `q_ẋ`、`q_θ̇` |
| 省力、动作更柔和 | 减小 `q_θ` 或增大 `r_weight` |
| 抖动/嗡鸣 | 增大 `r_weight`（=1 → 3~10），或降 `control_rate` |

经验：`q_θ : q_x ≈ 50:1` 起步；`r_weight` 从 1 开始，翻倍调。

改完注意闭环极点应该全部为负：

```bash
ros2 param set /leg_calc_lqr_controller q_diag "[20.0, 2000.0, 1.0, 1.0]"
# 参数回调会重新求解并在日志里打印名义高度处的 K
```

### 6.4 `u_max` 与饱和

`u_max`（默认 5.0）是控制量硬限幅。若 `/leg_calc/control` 长期贴边，说明：
- θ 零位没标定（最常见）；
- 增益过大（Q 太大/R 太小）；
- 电机力矩不足 → 应提高 `u_max`，但同时要检查机构。

调试时可临时调小 `u_max`（如 1.0）再逐步放开。

### 6.5 腿部几何与关节标定

上位机：`thigh`(l1 髋→膝)、`shank`(l2 膝→轮轴)、`com_offset`(髋→质心，质心在髋上方为正)，
`h = l1·cos(q_hip)+l2·cos(q_hip+q_knee)+com_offset`（左右腿取平均），并用 `h_min/h_max` 截断。

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

> 标定后，`thigh/shank/com_offset` 必须按**这个零位姿态**测量。

没标定时可先关闭高度调度：

```bash
ros2 param set /leg_calc_lqr_controller use_measured_height false
```

### 6.6 K(h) 二次拟合流程

**第 1 步：生成数据并拟合**

```bash
ros2 run leg_calc gain_table --ros-args \
  -p h_min:=0.18 -p h_max:=0.40 -p samples:=12 \
  -p cart_mass:=1.2 -p pendulum_mass:=2.5 -p com_inertia:=0.05 \
  -p q_diag:="[10.0, 500.0, 1.0, 1.0]" -p r_weight:=1.0
```

输出形如：

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

**第 2 步：填进 `config/lqr_params.yaml` 的 `k_coeffs`**（行序 = `[k_x, k_theta, k_x_dot, k_theta_dot]`，列序 = `[h², h, 1]`），并把 `gain_mode` 设为 `quadratic`。

**第 3 步：在线核对**（切换只看效果，不必重启）：

```bash
# 用拟合值
ros2 param set /leg_calc_lqr_controller k_coeffs "[0,-0,-3.162278, -5.0124,-1.9104,-27.2805, \
  -0.3654,-0.1540,-4.1653, -4.2589,-2.2130,-1.9559]"
# 切回现场解 LQR 对照（把 k_coeffs 全置 0 即可）
ros2 param set /leg_calc_lqr_controller k_coeffs "[0,0,0, 0,0,0, 0,0,0, 0,0,0]"
```

对比 `/leg_calc/gain` 与 `/leg_calc/com_height` 两组曲线，偏差应 < 1%。

> 备注：`k_x = -√(q_x/r_weight)` 与 h 无关，拟合出来 c2=c1=0 属正常。

### 6.7 轮子映射与指令语义

| 参数 | 默认 | 说明 |
|---|---|---|
| `wheel_cmd_mode` | `velocity` | `velocity`：u 乘增益当作轮速，**由上位机积分成角度指令**（固件轮环只用 `wheel.angle`）；`position`：u 直接当目标轮角 |
| `left_wheel_gain` / `right_wheel_gain` | 1.0 / 1.0 | u → 左右轮指令的比例；差速/配平用 |
| `left_invert` / `right_invert` | false | 轮子安装方向相反时置 true（同时影响 odom 与关节状态上报） |
| `max_wheel_omega` | 20.0 | 轮速指令限幅 [rad/s] |
| `max_command` | 5.0 | 进入节点前的 u 限幅 |
| `command_timeout` | 0.3 | 控制指令中断多久后转「轮子位置保持 + 关节失力」 |
| `joint_kp` / `joint_kd` | 0 / 0 | 4 个腿关节的刚度/阻尼；0 表示纯力矩（自由） |

改轮速前先确认 `odom` 里的 `x` 方向与实物一致（推车向前 `x` 增大），否则 `left/right_invert` 反了。

### 6.8 通信参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vid` / `pid` | 1155 / 22336 | 0x0483 / 0x5740，STM32 虚拟串口 |
| `interface` / `ep_in` / `ep_out` | 1 / 0x81 / 0x01 | CDC 数据接口与端点，一般不用改 |
| `send_rate` | 100 | 下行帧率，与固件 `state_send_task` 的 10 ms 对齐 |
| `wheel_radius` / `wheel_separation` | 0.065 / 0.20 | 轮半径、轮距（用于 odom 的 x 与 ωz） |

免 sudo 访问（可选）：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-wheel-leg.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 6.9 参数速查（`leg_calc/config/lqr_params.yaml`）

| 组 | 参数 | 默认 |
|---|---|---|
| 模型 | `cart_mass` / `pendulum_mass` / `com_length` / `com_inertia` | 1.2 / 2.5 / 0.28 / 0.05 |
| | `gravity` / `wheel_radius` / `wheel_inertia` / `input_mode` | 9.81 / 0.065 / 0.0008 / `wheel_torque` |
| 权重 | `q_diag` / `r_weight` | `[10,500,1,1]` / 1.0 |
| 高度调度 | `gain_mode` / `k_coeffs` / `thigh` / `shank` / `com_offset` | `quadratic` / 12 个 0 / 0.15 / 0.15 / 0.10 |
| | `h_min` / `h_max` / `use_measured_height` / `height_timeout` | 0.10 / 0.60 / true / 0.5 |
| 关节 | `leg_joint_names` / `joint_states_topic` | 4 个名字 / `joint_states` |
| 控制 | `control_rate` / `u_max` / `reference_state` | 200 / 5.0 / `[0,0,0,0]` |
| 姿态 | `theta_axis` / `theta_offset` / `imu_timeout` | `pitch` / 0.0 / 0.2 |

除 `control_rate`、`imu_topic`、`odom_topic`、`joint_states_topic` 外，其余都支持运行时 `ros2 param set` 热更新（参数回调里会重新解一次 LQR 并在日志打印新 K；非法值会被拒绝并返回原因）。

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `colcon build` 报 `Invalid email "“..."` | `package.xml` 里的邮箱被中文引号包住 | 改成 ASCII 引号 |
| 改包名后报 `source directory ... does not exist` | `build/<旧包名>` 里 CMakeCache 仍指向旧路径 | 删掉 `build/<旧包名>`、`install/<旧包名>` 再编译 |
| `无法打开 IMU 串口` | 设备没插 / 端口不对 / 无权限 | `ls /dev/ttyUSB*`；`-p port:=...`；`sudo usermod -aG dialout $USER` |
| `/odom` 没有数据 | `usb_drive` 没起来 / USB 未连 / 固件版本旧 | 看 `usb_drive_node` 日志；`ros2 topic hz /usb_drive/state` |
| `/com_height` 恒为 `l1+l2+offset` | 固件没上报关节角（旧版 `rob_state.joint` 全 0） | 更新固件（`state_send_task` 已补拷贝） |
| `control` 一启动就饱和 | `theta_offset` 没标定 / 增益过大 | 重新标定零位；临时把 `u_max` 调到 1 |
| 摆杆来回抖、嗡鸣 | 增益过高或噪声大 | 增大 `r_weight`；降 `control_rate`；检查 IMU 安装刚性 |
| 车往一个方向跑 | 轮子方向或 odom 符号错 | 校 `left_invert`/`right_invert`；查 `left/right_wheel_gain` |
| 关节数据乱跳 | 协议长度/对齐不一致 | 确认固件 `task.h` 与 `data_pack.h` 一致（编译期 `static_assert` 会先报） |
| 日志刷 `USB-CDC 打开失败` | 设备未插入（每 2 s 重试，属正常） | 插上即可，无需重启节点 |
| 编译卡在 `Failed to clone repository: ...spdlog.git`（HTTP 408） | FetchContent 从 GitHub 拉依赖时网络超时 | 按 5.0 的方法本地 clone 后传 `-DFETCHCONTENT_SOURCE_DIR_SPDLOG_LIB` / `..._CEREAL_LIB` |
| `Failed to find .../install/mujoco_ros2_control/share/.../package.sh` | 只 `--packages-select` 了别的包，`mujoco_ros2_control` 还没构建过 | 先 `colcon build --packages-select mujoco_ros2_control`，或直接全量 `colcon build` |
| MuJoCo 窗口里机器人一开始就倒 | 浮动基座 + 腿伸直，没有平衡控制 | 正常现象，见 8.5；可发位置指令摆屈膝姿态观察 |
| 发指令后关节不动 | MJCF 关节名与 URDF `<ros2_control>` 名字不一致（插件只按名字匹配，不报错） | 对照附录 C 的关节表；启动日志里 `added ... actuator for joint:` 应该打满 6 条 |
| 关节能量了但轮子不转 | `motor` actuator 被写成了 `position`（或加了 `kv`），识别成 POSITION 接口 | MJCF 里轮子用 `<motor>`；腿的 `<position>` **不要带 `kv`** |
| RViz 里机器人位姿不动 / 悬在空中 | URDF 描述不了浮动基座，`base_link` 被当成固定在世界原点 | 位姿以 MuJoCo 窗口为准；要联动需自己桥接 `odom → base_link` |
| 启动日志 `KDL does not support a root link with an inertia` | `base_link` 是根连杆且带惯量 | 预期告警，可忽略（URDF 头部注释里已说明） |

---

## 8. 已知限制 / 待办

1. 动力学是 **cart-pole 等效模型**：忽略腿关节主动力矩、轮地打滑、两轮差速带来的横滚/偏航自由度。
2. 固件轮环是「位置 → 速度」串级，LQR 的力矩只能经上位机积分成角度指令下发；若要真正力矩直通，需给 `Wheel_t` 增加 `torque` 字段或加模式字节。
3. M2006 首帧存在一圈（8192 counts ≈ 360°）绝对角歧义，上电瞬间固件位置环可能做一次大角度修正。
4. `usb_drive` 未发布 TF（`odom → base_link`），RViz 里需自行补 `static_transform_publisher`。
5. **仿真没有接 LQR 闭环**。`leg_calc` 需要 `/imu/data` 的 `orientation`（算 pitch）和 `/odom`，而 MuJoCo 侧只提供：
   - `/imu_site_imu_sensor/imu`：只有 `angular_velocity` 和 `linear_acceleration`，`orientation` 是单位四元数；
   - `/base_link_pose_sensor/pose`：世界位姿（`PoseStamped`），但 `leg_calc` 不订阅它。

   要闭环，需要写一个桥接节点：用 `pose` 的姿态与速度拼出 `sensor_msgs/Imu`（把四元数填进 `orientation`）+ `nav_msgs/Odometry`（`pose.position.x` 取机身世界 x，`twist.linear.x` 取 `qvel` 折算的线速度），再 remap 到 `imu/data` 和 `odom`。注意 `leg_calc` 的 θ 是「摆杆相对竖直向上」的角，与 `base_link` 的 pitch 相差一个常值，需要调 `theta_offset` 对齐。
6. **URDF 没有（也无法有）浮动基座**：`base_link` 就是根连杆，`world → base_link` 的 TF 不存在。RViz 里看到的是"固定在原点、只有关节在动"的机器人，真实位姿要看 MuJoCo 窗口。
7. **模型尺寸是估的**：`l1=0.15 / l2=0.15 / r=0.065 / 轮距 0.20` 与 `lqr_params.yaml` 一致，但机身 0.24×0.20×0.10、腿片厚度、防护架高度是按照片目测的。拿到实物尺寸后同时改 `wheel_leg.urdf` 和 `wheel_leg.xml`（两边必须同步，否则仿真动力学与 RViz 显示不一致）。
8. 仿真的腿关节是**位置伺服**（`kp=30`），不能像实机 RobStride 那样直接给力矩；要做腿力矩控制（如 VMC、全身控制），得把 MJCF 的 `position` actuator 改成 `motor`，并在 URDF 里把 `command_interface` 换成 `effort`。
9. **VMC 用的是五杆闭链几何，而 URDF/MJCF 还是串联 2R**（`base_link → 大腿 → 小腿 → 轮`）。也就是说 VMC 只能上实机，**在 MuJoCo 里跑会与模型不符**；要进仿真必须先把 URDF/MJCF 改成 `l1..l5` 的双髋闭链（两条腿各 2 个髋关节，共 4 个驱动关节，与原 `joint` 数量一致，名字可沿用）。
10. VMC 是纯 PD，**没有积分项**：静态支撑必然有下垂（`Δy ≈ 单腿负载 / kp_y`，默认约 5 cm）。要定高需要加重力前馈或积分项；`foot_target_y` 目前是手填常数，还没接姿态（roll）/高度指令。
11. 逆运动学在**某条 2R 子链接近完全伸直**时两组装配解会重合，固定分支参数会失效；此时应传 `q_seed`（实测关节角）按最近解跟踪，或干脆避开该区域。
12. 差速底盘只做**运动学分解**（开环）：`cmd_vel` 不闭环到实际轮速，打滑、负载差异、轮径误差都会造成航向漂移；需要走直线精度时要加轮速闭环或 IMU/视觉航向修正。
13. 差速量只在 `usb_drive` 的 `wheel_cmd_mode=velocity` 下生效（固件轮环是位置-速度串级，上位机把速度积分成角度）；`position` 模式下忽略 `wheel_target`。
14. 轮子目标 = 平衡量 `u×gain`（左右同向）+ 行驶量 `wheel_target`（左右差速）。**二者共用 `max_wheel_omega` 限幅**，所以大速度行驶时留给平衡的余量会变小；行驶速度上限要按平衡余量留够。

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

### 9.3 `vmc_params.yaml` 速查

| 组 | 参数 | 默认 |
|---|---|---|
| 几何 | `l1` / `l2` / `l3` / `l4` / `l5` | 0.10 / 0.20 / 0.20 / 0.10 / 0.10 |
| 装配 | `foot_branch` / `front_branch` / `rear_branch` | 1 / 1 / 1 |
| 刚度 | `kp_x` / `kp_y` | 60 / 300 N/m |
| 阻尼 | `kd_x` / `kd_y` | 3 / 12 N·s/m |
| 限幅 | `force_max_x` / `force_max_y` / `torque_max` | 40 / 150 N / 8 N·m |
| 目标 | `foot_target_x` / `foot_target_y` / `foot_vel_target_*` | 0.05 / 0.30 m / 0 |
| 输出 | `joint_torque_sign` / `control_rate` / `joint_state_timeout` | 4×1.0 / 200 Hz / 0.2 s |
| 接口 | `joint_states_topic` / `joint_target_topic` / `leg_joint_names` | `joint_states` / `joint_target` / 4 个腿关节名 |

除三个话题名外都支持 `ros2 param set` 热更新（几何非法、`kp/kd<0`、`torque_max<=0` 会被拒绝并返回原因）。

### 9.4 调参顺序

```
① 量 l1..l5（实测）→ 确认 α=β=90° 时足端位置与实机一致（启动日志会打印）
② 标定关节零位：此时 q=[90°, 90°] 应让两摆臂竖直向下
③ kp_y 从小到大（100 → 300 → 600），吊起机器看腿能否"撑住"
④ kd_y 抑制上下弹跳（约 0.2·sqrt(kp_y·m_单腿) 起调）
⑤ kp_x/kd_x 抑制前后晃动（一般比 y 轴软，避免和轮子 LQR 抢位置）
⑥ 力矩方向：若某关节"越用力越往错误方向走"，把 joint_torque_sign 对应位置 -1
⑦ 最后再放开 force_max / torque_max
```

**别和 LQR 抢**：VMC 管腿（`/joint_target` 的 effort），LQR 管轮（`/leg_calc/control`）。`kp_x` 太大时腿会把机身前后"锁死"，轮子的平衡动作会被抵消，表现为车不动、腿抖。

---

## 10. 差速底盘运动学

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

**正解**（轮速 → 底盘，`usb_drive` 的 odom 用的就是这组）：

```
v = (ω_L + ω_R)·r / 2
ω = (ω_R − ω_L)·r / L
```

**反解**（底盘 → 轮速，`diff_drive_node` 执行的部分）：

```
ω_L = (v − L/2·ω) / r
ω_R = (v + L/2·ω) / r
```

符号：`ω > 0` 左转（逆时针、yaw 增大，右轮快左轮慢）；`ω < 0` 右转；
`v = 0、ω ≠ 0` 是原地转；`ω_L = ω_R` 是直线。

限幅顺序（`wheelsFromBody`）：先把 `v`、`ω` 截到 `max_linear` / `max_angular`，
再算轮速；若某轮超 `max_wheel_speed`，左右轮**按同一比例缩放** —— 这样转向半径不变，
只是整体变慢（`diff_drive/state` 里报的就是缩放后的实际值，`saturated=1`）。

### 10.2 `diff_drive_params.yaml` 速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `wheel_separation` | 0.20 | L [m]，必须与 `usb_drive` 的一致 |
| `wheel_radius` | 0.065 | r [m]，必须与 `usb_drive` 的一致 |
| `max_linear` | 1.0 | `|v|` 限幅 [m/s]，`<=0` 不限 |
| `max_angular` | 3.0 | `|ω|` 限幅 [rad/s]，`<=0` 不限 |
| `max_wheel_speed` | 20.0 | 单轮限幅 [rad/s]，`<=0` 不限；不要超过 `usb_drive` 的 `max_wheel_omega` |
| `cmd_velocity_timeout` | 0.3 | `cmd_vel` 中断多久后置零 [s] |
| `publish_rate` | 50.0 | `/wheel_target` 发布频率 [Hz] |
| `cmd_vel_topic` / `wheel_target_topic` | `cmd_vel` / `wheel_target` | 话题名 |

几何与限幅参数支持 `ros2 param set` 热更新；话题名需要重启。

### 10.3 与平衡控制的关系

```
轮子目标速度 = u×gain（LQR 平衡，左右同向） + wheel_target（差速行驶，左右不同）
```

- 只有 LQR 时不发 `wheel_target`，行为与以前完全一致。
- 只有差速时（不跑 LQR），左右轮按差分轮纯行驶。
- 两者都在时就是"边走边自平衡"：平衡量消除俯仰，行驶量负责前进/转向。
- 任一路超时（默认 0.3 s）该路按 0 处理；两路都超时 → 轮子位置保持、关节失力。
- 使用前确认 `/odom` 的 `x` 方向与实物一致（推车向前 `x` 增大），否则 `left/right_invert` 反了，
  差速的转向也会跟着反。

### 10.4 最小闭环（可选）

`diff_drive_node` 是开环运动学。要压住直线漂移，可以再加一个轮速环：
订阅 `/joint_states` 里两个轮子的 `velocity`，用 PI 修正 `wheel_target`，或直接用
`/cmd_vel` 与 `/odom.twist` 的偏差修正 `v`。这一步没做，接口已经留好（`wheel_target` 就是轮速指令）。

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

**高度调度**：`h(q) = l1·cos(q_h) + l2·cos(q_h+q_k) + c`，`K_j(h) = c2·h² + c1·h + c0`。

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

## 附录 C：URDF / MJCF 约定

**关节表**（URDF 的 `<ros2_control>`、MJCF 的 `<joint>`、仿真控制器三处名字必须完全一致）

| 关节 | 类型 | 轴 | 限位 [rad] | 命令接口 | MJCF actuator |
|---|---|---|---|---|---|
| `leg_left_hip_joint` | revolute | `0 1 0` | [-1.30, 1.30] | position | `position` kp=30 |
| `leg_left_knee_joint` | revolute | `0 1 0` | [-2.60, 0.10] | position | `position` kp=30 |
| `leg_right_hip_joint` | revolute | `0 1 0` | [-1.30, 1.30] | position | `position` kp=30 |
| `leg_right_knee_joint` | revolute | `0 1 0` | [-2.60, 0.10] | position | `position` kp=30 |
| `wheel_left_joint` | continuous | `0 1 0` | — | effort | `motor` ±3 N·m |
| `wheel_right_joint` | continuous | `0 1 0` | — | effort | `motor` ±3 N·m |

**连杆对应**

| URDF link | MJCF body | 质量 [kg] | 说明 |
|---|---|---|---|
| `base_link` | `base_link`（带 `freejoint`） | 1.00 = 0.85 盒 + 4×0.02 立柱 + 4×0.0175 横杆 | 机身 + 顶部防护架 |
| `leg_left_thigh` / `leg_right_thigh` | 同名 | 0.35 = 0.05 髋块 + 0.30 大腿片 | 橙色片状连杆 |
| `leg_left_shank` / `leg_right_shank` | 同名 | 0.25 = 0.04 膝块 + 0.21 小腿片 | 橙色片状连杆 |
| `wheel_left` / `wheel_right` | 同名 | 0.20 = 0.18 胎 + 0.02 毂 | 毂纯视觉（`contype=0`） |
| `imu_link` | `imu_link`（内含 `imu_site`） | 0.01 | 防护架顶部，z=0.16 |

合计 3.0 kg：腿 + 机身 2.20 kg（≈ `pendulum_mass` 2.5），轮子 0.40 kg（计入 `cart_mass`）。

**零位与高度公式**

所有关节角 = 0 时大腿、小腿都竖直向下，轮心在髋轴正下方 0.30 m，整机直立时 `base_link` 离地 `0.30 + 0.065 = 0.365 m`（MJCF 里 `base_link` 的 `pos="0 0 0.365"` 就是这个值）。

这个零位约定让仿真姿态与 `leg_calc` 的高度公式直接对应：

```
h = l1*cos(q_hip) + l2*cos(q_hip + q_knee) + com_offset
  = 0.15*cos(q_hip) + 0.15*cos(q_hip+q_knee) + 0.10
```

腿伸直时 `h = 0.40`；实机的屈膝站姿（膝约 -0.55）对应 `h ≈ 0.365`。
实机静止是屈膝的，仿真里保持零位竖直、用位置指令摆站姿，是为了不破坏上式与 q 的一一对应。

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
