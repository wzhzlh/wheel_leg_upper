#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "usb_drive/cdc_trans.hpp"
#include "usb_drive/data_pack.h"

namespace
{

std::vector<uint8_t> makePattern(size_t count, uint8_t start = 0)
{
  std::vector<uint8_t> data(count);
  for (size_t i = 0; i < count; ++i) {
    data[i] = static_cast<uint8_t>(start + static_cast<uint8_t>(i & 0xFF));
  }
  return data;
}

}  // namespace

// 协议结构体必须与固件逐字节一致（固件未使用 #pragma pack）
TEST(DataPackLayout, MatchesFirmware)
{
  EXPECT_EQ(sizeof(usb_drive::Wheel_t), 8u);
  EXPECT_EQ(sizeof(usb_drive::JointTarget_t), 20u);
  EXPECT_EQ(sizeof(usb_drive::JointState_t), 48u);
  EXPECT_EQ(sizeof(usb_drive::RobotTarget_t), 96u);
  EXPECT_EQ(sizeof(usb_drive::RobotState_t), 208u);

  // 关节反馈中 uint64_t 会引入 4 B 填充，偏移必须与固件一致
  EXPECT_EQ(offsetof(usb_drive::JointState_t, error), 32u);
  EXPECT_EQ(offsetof(usb_drive::JointState_t, feedback), 40u);
  EXPECT_EQ(offsetof(usb_drive::RobotState_t, wheel), 0u);
  EXPECT_EQ(offsetof(usb_drive::RobotState_t, joint), 16u);
}

// 一次收到两帧（粘包）
TEST(FrameAssembler, SplitsCoalescedFrames)
{
  const size_t frame_size = sizeof(usb_drive::RobotState_t);
  std::vector<std::vector<uint8_t>> frames;
  usb_drive::FrameAssembler assembler(
    frame_size, [&frames](const uint8_t * frame, size_t size) {
      frames.emplace_back(frame, frame + size);
    });

  const auto payload = makePattern(frame_size * 2);
  assembler.feed(payload.data(), payload.size());

  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0], std::vector<uint8_t>(payload.begin(), payload.begin() + frame_size));
  EXPECT_EQ(frames[1], std::vector<uint8_t>(payload.begin() + frame_size, payload.end()));
  EXPECT_EQ(assembler.pendingBytes(), 0u);
}

// 一帧被拆成多段（拆包）
TEST(FrameAssembler, HandlesSplitFrames)
{
  const size_t frame_size = sizeof(usb_drive::RobotState_t);
  size_t calls = 0;
  usb_drive::FrameAssembler assembler(
    frame_size, [&calls, frame_size](const uint8_t *, size_t size) {
      EXPECT_EQ(size, frame_size);
      ++calls;
    });

  const auto payload = makePattern(frame_size);
  size_t offset = 0;
  for (const size_t chunk : {1u, 7u, 64u, 64u, 72u}) {
    assembler.feed(payload.data() + offset, chunk);
    offset += chunk;
  }

  EXPECT_EQ(offset, frame_size);
  EXPECT_EQ(calls, 1u);
  EXPECT_EQ(assembler.pendingBytes(), 0u);
}

// 不足一帧时保留残余
TEST(FrameAssembler, KeepsIncompleteFrame)
{
  const size_t frame_size = sizeof(usb_drive::RobotState_t);
  size_t calls = 0;
  usb_drive::FrameAssembler assembler(frame_size, [&calls](const uint8_t *, size_t) {++calls;});

  const auto payload = makePattern(frame_size + 10);
  assembler.feed(payload.data(), payload.size());

  EXPECT_EQ(calls, 1u);
  EXPECT_EQ(assembler.pendingBytes(), 10u);

  assembler.reset();
  EXPECT_EQ(assembler.pendingBytes(), 0u);
}

// 回调内 reset() 不能破坏内部状态（失步后重新同步的路径）
TEST(FrameAssembler, ResetInsideCallbackIsSafe)
{
  const size_t frame_size = sizeof(usb_drive::RobotState_t);
  size_t calls = 0;
  usb_drive::FrameAssembler * self = nullptr;
  usb_drive::FrameAssembler assembler(
    frame_size, [&calls, &self](const uint8_t *, size_t) {
      ++calls;
      self->reset();
    });
  self = &assembler;

  const auto payload = makePattern(frame_size * 3);
  assembler.feed(payload.data(), payload.size());

  EXPECT_EQ(calls, 1u);
  EXPECT_EQ(assembler.pendingBytes(), 0u);
}
