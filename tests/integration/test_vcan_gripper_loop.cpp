#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "piper/gripper/gripper_client.hpp"

namespace {

int g_failures = 0;

#define EXPECT_TRUE(cond, msg)                                     \
  do {                                                              \
    if (!(cond)) {                                                  \
      std::cerr << "[FAIL] " << msg << "\n";                     \
      ++g_failures;                                                 \
    }                                                               \
  } while (0)

int OpenCanSocket(const std::string& ifname, canid_t filter_id, int timeout_ms) {
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return -1;
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    return -1;
  }

  can_filter filter{};
  filter.can_id = filter_id;
  filter.can_mask = CAN_SFF_MASK;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    ::close(fd);
    return -1;
  }

  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    ::close(fd);
    return -1;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

bool SendFrame(int fd, canid_t can_id, const uint8_t data[8]) {
  can_frame frame{};
  frame.can_id = can_id;
  frame.can_dlc = 8;
  for (int i = 0; i < 8; ++i) {
    frame.data[i] = data[i];
  }
  const ssize_t n = ::write(fd, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

int32_t ToInt32Big(const uint8_t* data) {
  const uint32_t v = (static_cast<uint32_t>(data[0]) << 24) |
                     (static_cast<uint32_t>(data[1]) << 16) |
                     (static_cast<uint32_t>(data[2]) << 8) |
                     static_cast<uint32_t>(data[3]);
  return static_cast<int32_t>(v);
}

}  // namespace

int main() {
  const std::string ifname = "vcan0";
  if (if_nametoindex(ifname.c_str()) == 0U) {
    std::cerr << "[SKIP] vcan0 is not available. run tools/setup_vcan.sh up\n";
    return 0;
  }

  piper::gripper::Config cfg;
  cfg.can_ifname = ifname;
  cfg.recv_timeout_ms = 100;
  cfg.monitor_period_ms = 50;
  cfg.fps_period_ms = 100;
  cfg.is_ok_window = 5;

  piper::gripper::GripperClient client(cfg);
  auto start_ret = client.Start();
  EXPECT_TRUE(start_ret.code == piper::gripper::ErrorCode::Ok, "Start should succeed on vcan0");
  if (start_ret.code != piper::gripper::ErrorCode::Ok) {
    return 1;
  }

  const int capture_fd = OpenCanSocket(ifname, 0x159, 500);
  EXPECT_TRUE(capture_fd >= 0, "capture socket open failed");

  piper::gripper::GripperCommand cmd;
  cmd.angle_0p001mm = 70000;
  cmd.effort_0p001Nm = 1000;
  cmd.status = piper::gripper::StatusCode::Enable;
  cmd.set_zero = piper::gripper::SetZero::Invalid;

  auto send_ret = client.GripperCtrl(cmd);
  EXPECT_TRUE(send_ret.code == piper::gripper::ErrorCode::Ok, "GripperCtrl should send");

  can_frame captured{};
  const ssize_t n = (capture_fd >= 0) ? ::read(capture_fd, &captured, sizeof(captured)) : -1;
  EXPECT_TRUE(n == static_cast<ssize_t>(sizeof(captured)), "should capture 0x159 frame");
  if (n == static_cast<ssize_t>(sizeof(captured))) {
    EXPECT_TRUE(captured.can_id == 0x159, "captured frame can_id mismatch");
    EXPECT_TRUE(ToInt32Big(captured.data) == 70000, "captured angle mismatch");
    EXPECT_TRUE(captured.data[4] == 0x03 && captured.data[5] == 0xE8, "captured effort mismatch");
    EXPECT_TRUE(captured.data[6] == 0x01, "captured status mismatch");
  }

  const int inject_fd = OpenCanSocket(ifname, 0x159, 500);
  EXPECT_TRUE(inject_fd >= 0, "inject socket open failed");

  const uint8_t feedback_payload[8] = {
      0x00, 0x00, 0x75, 0x30,  // 30000
      0x00, 0x64,              // 100
      0xC1,                    // bit6+bit7+bit0
      0x00,
  };
  const bool injected = (inject_fd >= 0) ? SendFrame(inject_fd, 0x2A8, feedback_payload) : false;
  EXPECT_TRUE(injected, "inject 0x2A8 frame failed");

  piper::gripper::Result<piper::gripper::GripperState> state;
  bool got_state = false;
  for (int i = 0; i < 20; ++i) {
    state = client.GetArmGripperMsgs();
    if (state.code == piper::gripper::ErrorCode::Ok && state.value.timestamp_sec > 0.0) {
      got_state = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  EXPECT_TRUE(got_state, "should receive gripper feedback state");
  if (got_state) {
    EXPECT_TRUE(state.value.angle_0p001mm == 30000, "state angle mismatch");
    EXPECT_TRUE(state.value.effort_0p001Nm == 100, "state effort mismatch");
    EXPECT_TRUE(state.value.status_code == 0xC1, "state status code mismatch");
    EXPECT_TRUE(state.value.foc.voltage_too_low, "state foc bit0 mismatch");
    EXPECT_TRUE(state.value.foc.driver_enable_status, "state foc bit6 mismatch");
    EXPECT_TRUE(state.value.foc.homing_status, "state foc bit7 mismatch");
  }

  client.EnableSDKGripperLimit(true);
  (void)client.SetSDKGripperRangeParam(0.0, 0.07);
  const uint8_t clamp_payload[8] = {
      0x00, 0x01, 0x86, 0xA0,  // 100000
      0x00, 0x10,
      0x00,
      0x00,
  };
  EXPECT_TRUE(SendFrame(inject_fd, 0x2A8, clamp_payload), "inject clamp payload failed");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  state = client.GetArmGripperMsgs();
  EXPECT_TRUE(state.value.angle_0p001mm == 70000, "clamp should limit angle to 70000");

  client.EnableSDKGripperLimit(false);
  client.EnableFilterAbnormalData(true);
  const auto before = client.GetArmGripperMsgs();
  const uint8_t abnormal_payload[8] = {
      0x00, 0x03, 0x0D, 0x40,  // 200000
      0x00, 0x20,
      0x00,
      0x00,
  };
  EXPECT_TRUE(SendFrame(inject_fd, 0x2A8, abnormal_payload), "inject abnormal payload failed");
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto after = client.GetArmGripperMsgs();
  EXPECT_TRUE(before.value.timestamp_sec == after.value.timestamp_sec,
              "abnormal frame should be filtered and not update timestamp");

  EXPECT_TRUE(client.IsOk(), "isOk should be true after recent traffic");
  std::this_thread::sleep_for(std::chrono::milliseconds(2200));
  EXPECT_TRUE(!client.IsOk(), "isOk should become false when fps window stays zero");

  auto stop_ret = client.Stop();
  EXPECT_TRUE(stop_ret.code == piper::gripper::ErrorCode::Ok, "Stop should succeed");

  if (capture_fd >= 0) {
    ::close(capture_fd);
  }
  if (inject_fd >= 0) {
    ::close(inject_fd);
  }

  if (g_failures == 0) {
    std::cout << "[PASS] integration_vcan_gripper_loop\n";
    return 0;
  }
  std::cerr << "[FAIL] integration_vcan_gripper_loop failures=" << g_failures << "\n";
  return 1;
}

