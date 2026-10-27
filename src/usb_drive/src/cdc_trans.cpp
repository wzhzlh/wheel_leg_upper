// USB CDC 传输实现，移植自 RC2026_RoboticArm2/robot_driver/src/cdc_trans.cpp
//
// 相对原版的改动：
//   1. 接口号 / 收发端点改为参数，默认 1 / 0x81 / 0x01
//   2. close() 先取消传输并等待取消回调完成，再释放 transfer，避免释放后
//      仍有回调访问缓冲区导致的 use-after-free
//   3. 发送失败时识别 LIBUSB_ERROR_NO_DEVICE 直接标记断开
//   4. 首次打开失败后由 processOnce() 自动重试，不依赖热插拔事件
//   5. 修正拼写：regeiser_recv_cb -> registerRecvCallback

#include "usb_drive/cdc_trans.hpp"

#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace usb_drive
{
namespace
{

rclcpp::Logger cdcLogger()
{
  return rclcpp::get_logger("usb_drive_cdc");
}

rclcpp::Clock & steadyClock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

}  // namespace

CdcTrans::CdcTrans()
{
  const int rc = libusb_init(&ctx_);
  if (rc != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(cdcLogger(), "libusb 初始化失败: %s", libusb_error_name(rc));
    ctx_ = nullptr;
  }
#if LIBUSB_API_VERSION >= 0x01000106
  else {
    libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
  }
#endif
}

CdcTrans::~CdcTrans()
{
  handling_events_ = false;
  close();
  if (hotplug_registered_ && ctx_) {
    libusb_hotplug_deregister_callback(ctx_, hotplug_handle_);
    hotplug_registered_ = false;
  }
  if (ctx_) {
    libusb_exit(ctx_);
    ctx_ = nullptr;
  }
}

bool CdcTrans::open(
  uint16_t vid, uint16_t pid, int interface_number, uint8_t ep_in, uint8_t ep_out)
{
  last_vid_ = vid;
  last_pid_ = pid;
  interface_number_ = interface_number;
  ep_in_ = ep_in;
  ep_out_ = ep_out;

  if (!ctx_) {
    RCLCPP_ERROR(cdcLogger(), "libusb 上下文不可用，无法打开 CDC 设备");
    return false;
  }

  if (handle_ || recv_transfer_) {
    close();
  }

  handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
  if (!handle_) {
    RCLCPP_WARN_THROTTLE(
      cdcLogger(), steadyClock(), 2000,
      "USB-CDC 设备 %04x:%04x 打开失败（未插入或无 udev 权限）", vid, pid);
    return false;
  }

  // 分离内核驱动（cdc_acm 会抢占接口）
  if (libusb_kernel_driver_active(handle_, interface_number_) == 1) {
    const int ret = libusb_detach_kernel_driver(handle_, interface_number_);
    RCLCPP_INFO(cdcLogger(), "卸载内核驱动接口%d，返回%d", interface_number_, ret);
  }

  int ret = libusb_claim_interface(handle_, interface_number_);
  if (ret != LIBUSB_SUCCESS) {
    RCLCPP_ERROR(
      cdcLogger(), "获取接口%d失败: %s", interface_number_, libusb_error_name(ret));
    libusb_close(handle_);
    handle_ = nullptr;
    return false;
  }
  RCLCPP_INFO(cdcLogger(), "获取接口%d成功", interface_number_);

  recv_transfer_ = libusb_alloc_transfer(0);
  if (!recv_transfer_) {
    RCLCPP_ERROR(cdcLogger(), "分配异步传输结构体失败");
    libusb_release_interface(handle_, interface_number_);
    libusb_close(handle_);
    handle_ = nullptr;
    return false;
  }

  libusb_fill_bulk_transfer(
    recv_transfer_, handle_, ep_in_, rx_buffer_, sizeof(rx_buffer_),
    &CdcTrans::onTransfer, this, 0);

  if (!hotplug_registered_ && libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    const int rc = libusb_hotplug_register_callback(
      ctx_,
      static_cast<libusb_hotplug_event>(
        LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
      LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
      &CdcTrans::onHotplugCallback, this, &hotplug_handle_);
    if (rc == LIBUSB_SUCCESS) {
      hotplug_registered_ = true;
    } else {
      RCLCPP_WARN(cdcLogger(), "热插拔回调注册失败: %s", libusb_error_name(rc));
    }
  }

  recv_pending_ = true;
  ret = libusb_submit_transfer(recv_transfer_);
  if (ret != LIBUSB_SUCCESS) {
    recv_pending_ = false;
    RCLCPP_WARN(cdcLogger(), "提交接收请求失败: %s", libusb_error_name(ret));
  }

  disconnected_ = false;
  handling_events_ = true;
  need_reconnect_ = false;
  RCLCPP_INFO(
    cdcLogger(), "USB-CDC 已打开 %04x:%04x 接口%d EP_IN=0x%02x EP_OUT=0x%02x",
    vid, pid, interface_number_, ep_in_, ep_out_);
  return true;
}

void CdcTrans::close()
{
  if (recv_transfer_) {
    if (recv_pending_) {
      libusb_cancel_transfer(recv_transfer_);
      // 等待取消回调把 recv_pending_ 清掉，避免释放仍在使用的传输结构体
      for (int i = 0; i < 20 && recv_pending_; ++i) {
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 5000;
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
      }
    }
    libusb_free_transfer(recv_transfer_);
    recv_transfer_ = nullptr;
  }

  if (handle_) {
    libusb_release_interface(handle_, interface_number_);
    libusb_close(handle_);
    handle_ = nullptr;
  }
  disconnected_ = true;
}

int CdcTrans::send(const uint8_t * data, int size, unsigned int timeout_ms)
{
  if (!handle_ || disconnected_) {
    return -2;
  }

  int transferred = 0;
  const int rc = libusb_bulk_transfer(
    handle_, ep_out_, const_cast<uint8_t *>(data), size, &transferred, timeout_ms);

  if (rc != LIBUSB_SUCCESS) {
    if (rc == LIBUSB_ERROR_NO_DEVICE || rc == LIBUSB_ERROR_IO) {
      disconnected_ = true;
    }
    RCLCPP_WARN_THROTTLE(
      cdcLogger(), steadyClock(), 1000, "USB-CDC 发送失败: size=%d, 实际=%d, %s",
      size, transferred, libusb_error_name(rc));
    return -1;
  }
  return transferred;
}

void CdcTrans::registerRecvCallback(RecvCallback recv_cb)
{
  recv_cb_ = std::move(recv_cb);
}

void LIBUSB_CALL CdcTrans::onTransfer(libusb_transfer * transfer)
{
  auto * self = static_cast<CdcTrans *>(transfer->user_data);
  self->recv_pending_ = false;

  if (!self->handling_events_ || self->recv_transfer_ == nullptr) {
    return;
  }

  if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
    // 超时/取消属正常退出路径，其余状态视为掉线
    if (transfer->status != LIBUSB_TRANSFER_TIMED_OUT &&
      transfer->status != LIBUSB_TRANSFER_CANCELLED)
    {
      self->disconnected_ = true;
    }
    return;
  }

  if (self->recv_cb_ && transfer->actual_length > 0) {
    self->recv_cb_(transfer->buffer, static_cast<int>(transfer->actual_length));
  }

  if (!self->handling_events_) {
    return;
  }
  self->recv_pending_ = true;
  if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
    self->recv_pending_ = false;
    self->disconnected_ = true;
  }
}

void CdcTrans::processOnce()
{
  if (!ctx_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return;
  }

  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 50000;  // 50 ms
  libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

  if (disconnected_ && handle_) {
    close();
  }

  // 掉线后或首次打开失败后，每 500 ms 重试一次
  if ((need_reconnect_ || handle_ == nullptr) && last_vid_ != 0 && handling_events_) {
    need_reconnect_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!open(last_vid_, last_pid_, interface_number_, ep_in_, ep_out_)) {
      need_reconnect_ = true;
    }
  }
}

void CdcTrans::onHotplug(libusb_hotplug_event event)
{
  if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
    disconnected_ = true;
  } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
    need_reconnect_ = true;
  }
}

int LIBUSB_CALL CdcTrans::onHotplugCallback(
  libusb_context *, libusb_device *, libusb_hotplug_event event, void * user_data)
{
  static_cast<CdcTrans *>(user_data)->onHotplug(event);
  return 0;
}

// ---------------------------- FrameAssembler ----------------------------

FrameAssembler::FrameAssembler(size_t frame_size, FrameCallback callback)
: frame_size_(frame_size), callback_(std::move(callback))
{
  if (frame_size_ == 0) {
    frame_size_ = 1;  // 防御：避免除零/死循环
  }
  buffer_.reserve(frame_size_ * 4);
}

void FrameAssembler::feed(const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  while (buffer_.size() >= frame_size_) {
    // 先切出帧并移除，再回调：回调内 reset() 也不会破坏迭代状态
    std::vector<uint8_t> frame(
      buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size_));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size_));
    if (callback_) {
      callback_(frame.data(), frame.size());
    }
  }

  // 长期凑不满或异常数据堆积时清空，等待重新同步
  if (buffer_.size() > 4 * frame_size_) {
    buffer_.clear();
  }
}

void FrameAssembler::reset()
{
  buffer_.clear();
}

}  // namespace usb_drive
