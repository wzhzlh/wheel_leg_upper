#ifndef USB_DRIVE__DATA_PACK_H_
#define USB_DRIVE__DATA_PACK_H_

// 上位机 <-> 下位机 通信协议
//
// 本文件与下位机固件严格对齐（stm32/one/task/task.h + Core/Inc/RobStride2.h）：
//   * 固件没有使用 #pragma pack，结构体依赖自然对齐，
//     因此这里同样不做打包，并用 static_assert 校验长度与关键偏移；
//   * 固件按“结构体裸内存”收发，无包头/magic/包类型字段，
//     上位机需按固定长度拆帧（RobotTarget_t 96 B / RobotState_t 208 B）。
//
// 固件中的对应关系：
//   RobotTarget_t : usb_target_received() -> memcpy(&rob_target, ...)
//   RobotState_t  : state_send_task()    -> CDC_Transmit_FS(&rob_state, sizeof(rob_state))

#include <stdint.h>

#ifdef __cplusplus
namespace usb_drive
{
#endif

/// 轮子目标 / 反馈（M2006 + 位置-速度串级 PID）
typedef struct {
  float angle;   ///< 角度 [rad]
  float omega;   ///< 角速度 [rad/s]
} Wheel_t;

/// 关节目标（RobStride 运动控制：τ = kp*(angle-θ) + kd*(omega-ω) + torque）
typedef struct {
  float torque;  ///< 前馈力矩 [N*m]
  float angle;   ///< 目标角度 [rad]
  float omega;   ///< 目标角速度 [rad/s]
  float kp;      ///< 位置刚度
  float kd;      ///< 阻尼
} JointTarget_t;

/// 关节反馈（对应固件 RobStrideState_t，注意 uint64_t 会引入 4 B 填充）
typedef struct {
  float rad;          ///< 关节角度 [rad]
  float omega;        ///< 关节角速度 [rad/s]
  float torque;       ///< 输出力矩 [N*m]
  float iqf;          ///< 电流 [A]
  float r;            ///< 相电阻参数
  float temperature;  ///< 温度 [℃]
  float vbus;         ///< 母线电压 [V]
  uint64_t error;     ///< 故障码
  uint8_t feedback;   ///< 反馈类型（0 失能 / 1 使能 ...）
} JointState_t;

/// 上位机 -> 下位机（96 B）
typedef struct {
  Wheel_t wheel[2];
  JointTarget_t joint[4];
} RobotTarget_t;

/// 下位机 -> 上位机（208 B）
typedef struct {
  Wheel_t wheel[2];
  JointState_t joint[4];
} RobotState_t;

#ifdef __cplusplus
}  // namespace usb_drive
#endif

#endif  // USB_DRIVE__DATA_PACK_H_
