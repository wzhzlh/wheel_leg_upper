#ifndef USB_DRIVE__DATA_PACK_H_
#define USB_DRIVE__DATA_PACK_H_

// 上位机 <-> 下位机 通信协议
//
// 本文件与下位机固件严格对齐（stm32/one/task/task.h + Core/Inc/RobStride2.h）：
//   * 固件没有使用 #pragma pack，结构体依赖自然对齐，
//     因此这里同样不做打包，并用 static_assert 校验长度与关键偏移；
//   * 固件按“结构体裸内存”收发，无包头/magic/包类型字段，
//     上位机需按固定长度拆帧（RobotTarget_t 104 B / RobotState_t 208 B）。
//
// 固件中的对应关系：
//   RobotTarget_t : usb_target_received() -> memcpy(&rob_target, ...)
//   RobotState_t  : state_send_task()    -> CDC_Transmit_FS(&rob_state, sizeof(rob_state))

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
namespace usb_drive
{
#endif

/// 轮子反馈（M2006 编码器）
typedef struct
{
  float angle;   ///< 角度 [rad]
  float omega;   ///< 角速度 [rad/s]
} Wheel_t;

/// 轮子目标（上位机 -> 下位机）
/// 下位机是**纯力矩直通**（task.c wheel_control_task，没有位置/速度环）：
///   current = clamp(torque * WHEEL_COUNTS_PER_NM(4500), ±16000)，angle/omega 不参与
typedef struct
{
  float angle;   ///< 保留字段 [rad]（上位机按 omega 积分填，固件不读）
  float omega;   ///< 保留字段 [rad/s]（/wheel_target，固件不读；仅仿真桥使用）
  float torque;  ///< 力矩指令 [N*m]（实机主通道；0 = 轮子不出力）
} WheelTarget_t;

/// 关节目标（RobStride 运动控制：τ = kp*(angle-θ) + kd*(omega-ω) + torque）
typedef struct
{
  float torque;  ///< 前馈力矩 [N*m]
  float angle;   ///< 目标角度 [rad]
  float omega;   ///< 目标角速度 [rad/s]
  float kp;      ///< 位置刚度
  float kd;      ///< 阻尼
} JointTarget_t;

/// 关节反馈（对应固件 RobStrideState_t，注意 uint64_t 会引入 4 B 填充）
typedef struct
{
  float rad;          ///< 关节角度 [rad]
  float omega;        ///< 关节角速度 [rad/s]
  float torque;       ///< 输出力矩 [N*m]
  float iqf;          ///< 电流 [A]
  float r;            ///< 相电阻参数
  float temperature;  ///< 温度 [℃]
  float vbus;         ///< 母线电压 [V]
  uint64_t error;     ///< 故障码
  /// 状态字（位标志，不是 0~8 枚举）：实测电机未使能时上报 0x80，
  /// 仅用于诊断，上位机不得据此判定帧非法
  uint8_t feedback;
} JointState_t;

/// 上位机 -> 下位机（104 B）
typedef struct
{
  WheelTarget_t wheel[2];  ///< 24 B（12 B/轮）
  JointTarget_t joint[4];  ///< 80 B（20 B/关节）
} RobotTarget_t;

/// 下位机 -> 上位机（208 B）
typedef struct
{
  Wheel_t wheel[2];
  JointState_t joint[4];
} RobotState_t;

#ifdef __cplusplus
// 与固件 task.h 一一对应的长度/偏移校验：任一处不一致就在编译期报错
static_assert(sizeof(Wheel_t) == 8, "Wheel_t 必须 8 B");
static_assert(sizeof(WheelTarget_t) == 12, "WheelTarget_t 必须 12 B");
static_assert(sizeof(JointTarget_t) == 20, "JointTarget_t 必须 20 B");
static_assert(sizeof(RobotTarget_t) == 104, "RobotTarget_t 必须 104 B");
static_assert(offsetof(RobotTarget_t, joint) == 24, "joint 偏移必须为 24");
static_assert(sizeof(RobotState_t) == 208, "RobotState_t 必须 208 B");

}  // namespace usb_drive
#endif

#endif  // USB_DRIVE__DATA_PACK_H_
