#ifndef USB_DRIVE__CDC_TRANS_HPP_
#define USB_DRIVE__CDC_TRANS_HPP_

// USB CDC（虚拟串口）异步收发封装 + 定长帧重组。
//
// CdcTrans 特点：
//   * 接收：libusb 异步 bulk 传输 + 回调，循环重提交，不阻塞主线程
//   * 发送：同步 bulk 传输（下位机控制周期内足够快）
//   * 热插拔：注册 libusb hotplug 回调，掉线置位、上电自动重连
//   * 事件循环：由独立线程持续调用 processOnce()
//
// FrameAssembler 特点：
//   固件按“结构体裸内存”收发，没有包头/长度字段，而 USB bulk 会出现
//   粘包（一次收到多帧）和拆包（一帧分多次到达），需要按固定帧长累积、切分。

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <vector>

#include <libusb-1.0/libusb.h>

namespace usb_drive
{

class CdcTrans
{
public:
  using RecvCallback = std::function<void(const uint8_t * data, int size)>;

  CdcTrans();
  ~CdcTrans();

  CdcTrans(const CdcTrans &) = delete;
  CdcTrans & operator=(const CdcTrans &) = delete;

  /**
   * @brief 按 VID/PID 打开 CDC 设备并开始异步接收
   * @param interface_number CDC 数据接口号（STM32 USB-CDC 通常为 1）
   * @param ep_in  批量输入端点（设备 -> 主机），默认 0x81
   * @param ep_out 批量输出端点（主机 -> 设备），默认 0x01
   */
  bool open(
    uint16_t vid, uint16_t pid, int interface_number = 1,
    uint8_t ep_in = 0x81, uint8_t ep_out = 0x01);

  /// 关闭设备并释放传输结构体（可重复调用）
  void close();

  bool isOpen() const {return handle_ != nullptr && !disconnected_;}

  /// 发送一段裸数据，返回实际发送字节数；-1 发送失败，-2 设备未连接
  int send(const uint8_t * data, int size, unsigned int timeout_ms = 5);

  /// 发送一个按 1 字节对齐的协议结构体
  template<typename T>
  bool sendStruct(const T & pack, unsigned int timeout_ms = 5)
  {
    static_assert(std::is_standard_layout<T>::value, "数据包必须是标准布局结构体");
    static_assert(std::is_trivial<T>::value, "数据包必须是可平凡复制的结构体");
    return send(reinterpret_cast<const uint8_t *>(&pack), sizeof(T), timeout_ms) ==
           static_cast<int>(sizeof(T));
  }

  /// 注册接收回调（在 USB 事件线程中调用，注意线程安全）
  void registerRecvCallback(RecvCallback recv_cb);

  /// 处理一次 USB 事件，需由独立线程循环调用
  void processOnce();

private:
  void onHotplug(libusb_hotplug_event event);

  static void LIBUSB_CALL onTransfer(libusb_transfer * transfer);
  static int LIBUSB_CALL onHotplugCallback(
    libusb_context * ctx, libusb_device * device, libusb_hotplug_event event, void * user_data);

  uint16_t last_vid_{0};
  uint16_t last_pid_{0};
  int interface_number_{1};
  uint8_t ep_in_{0x81};
  uint8_t ep_out_{0x01};

  RecvCallback recv_cb_;
  uint8_t rx_buffer_[2048]{};

  std::atomic_bool disconnected_{true};
  std::atomic_bool need_reconnect_{false};
  std::atomic_bool handling_events_{true};
  std::atomic_bool recv_pending_{false};

  libusb_transfer * recv_transfer_{nullptr};
  libusb_context * ctx_{nullptr};
  libusb_device_handle * handle_{nullptr};
  libusb_hotplug_callback_handle hotplug_handle_{0};
  bool hotplug_registered_{false};
};

/**
 * @brief 定长帧重组：把 CdcTrans 收到的字节流切成固定长度的帧
 *
 * 典型用法：FrameAssembler(sizeof(RobotState_t), ...) 作为 CdcTrans::registerRecvCallback
 * 的处理逻辑，凑满 208 B 就回调一次。
 */
class FrameAssembler
{
public:
  using FrameCallback = std::function<void(const uint8_t * frame, size_t size)>;

  FrameAssembler(size_t frame_size, FrameCallback callback);

  /// 追加一段接收数据，凑满一帧就回调一次（回调内可安全调用 reset()）
  void feed(const uint8_t * data, size_t size);

  /// 清空累积数据（失步或解析出错时调用）
  void reset();

  size_t pendingBytes() const {return buffer_.size();}
  size_t frameSize() const {return frame_size_;}

private:
  size_t frame_size_;
  FrameCallback callback_;
  std::vector<uint8_t> buffer_;
};

}  // namespace usb_drive

#endif  // USB_DRIVE__CDC_TRANS_HPP_
