#include <array>
#include <cstdint>
#include <iostream>

#include <linux/can.h>

#include "piper/gripper/gripper_client.hpp"
#include "src/internal/protocol_codec.hpp"

using piper::gripper::ErrorCode;
using piper::gripper::GripperClient;
using piper::gripper::GripperCommand;
using piper::gripper::SetZero;
using piper::gripper::StatusCode;
using piper::gripper::internal::DecodedGripperFeedback;
using piper::gripper::internal::ProtocolCodec;

namespace {

int g_failures = 0;

#define EXPECT_TRUE(cond, msg)                                     \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::cerr << "[FAIL] " << msg << "\n";                     \
      ++g_failures;                                                 \
    }                                                               \
  } while (0)

void TestEncode0x159NormalCase() {
  GripperCommand cmd;
  cmd.angle_0p001mm = -123456;
  cmd.effort_0p001Nm = 3210;
  cmd.status = StatusCode::Enable;
  cmd.set_zero = SetZero::SetZero;

  const auto payload = ProtocolCodec::EncodeGripperCtrl(cmd);
  const std::array<uint8_t, 8> expected = {0xFF, 0xFE, 0x1D, 0xC0, 0x0C, 0x8A, 0x01, 0xAE};
  EXPECT_TRUE(payload == expected, "Encode_0x159_NormalCase bytes mismatch");
}

void TestDecode0x2A8NormalCase() {
  can_frame frame{};
  frame.can_id = piper::gripper::internal::kCanIdGripperFeedback;
  frame.can_dlc = 8;
  frame.data[0] = 0x00;
  frame.data[1] = 0x00;
  frame.data[2] = 0x30;
  frame.data[3] = 0x39;  // 12345
  frame.data[4] = 0xFB;
  frame.data[5] = 0x2E;  // -1234
  frame.data[6] = 0x55;
  frame.data[7] = 0x00;

  DecodedGripperFeedback out;
  const bool ok = ProtocolCodec::DecodeGripperFeedback(frame, &out);
  EXPECT_TRUE(ok, "Decode_0x2A8_NormalCase decode failed");
  EXPECT_TRUE(out.angle_0p001mm == 12345, "Decode_0x2A8_NormalCase angle mismatch");
  EXPECT_TRUE(out.effort_0p001Nm == -1234, "Decode_0x2A8_NormalCase effort mismatch");
  EXPECT_TRUE(out.status_code == 0x55, "Decode_0x2A8_NormalCase status mismatch");
}

void TestDecode0x2A8SignedBoundary() {
  can_frame frame{};
  frame.can_id = piper::gripper::internal::kCanIdGripperFeedback;
  frame.can_dlc = 8;
  frame.data[0] = 0x80;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;  // int32 min
  frame.data[4] = 0x80;
  frame.data[5] = 0x00;  // int16 min
  frame.data[6] = 0x00;

  DecodedGripperFeedback out;
  const bool ok = ProtocolCodec::DecodeGripperFeedback(frame, &out);
  EXPECT_TRUE(ok, "Decode_0x2A8_SignedBoundary decode failed");
  EXPECT_TRUE(out.angle_0p001mm == static_cast<int32_t>(0x80000000u), "Decode_0x2A8_SignedBoundary angle mismatch");
  EXPECT_TRUE(out.effort_0p001Nm == static_cast<int16_t>(0x8000u), "Decode_0x2A8_SignedBoundary effort mismatch");
}

void TestEncode0x159InvalidEffort() {
  GripperClient client;
  GripperCommand cmd;
  cmd.effort_0p001Nm = 5001;
  const auto ret = client.GripperCtrl(cmd);
  EXPECT_TRUE(ret.code == ErrorCode::InvalidArgument, "Encode_0x159_InvalidEffort expected InvalidArgument");
}

}  // namespace

int main() {
  TestEncode0x159NormalCase();
  TestDecode0x2A8NormalCase();
  TestDecode0x2A8SignedBoundary();
  TestEncode0x159InvalidEffort();

  if (g_failures == 0) {
    std::cout << "[PASS] unit_protocol_codec\n";
    return 0;
  }
  std::cerr << "[FAIL] unit_protocol_codec failures=" << g_failures << "\n";
  return 1;
}

