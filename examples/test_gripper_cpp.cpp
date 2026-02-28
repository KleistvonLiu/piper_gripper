#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "piper/gripper/gripper_client.hpp"

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop.store(true); }

void PrintState(const piper::gripper::Result<piper::gripper::GripperState>& state,
                const char* prefix = "") {
  if (state.code != piper::gripper::ErrorCode::Ok) {
    std::cout << prefix << "读取失败: " << state.message << "\n";
    return;
  }
  const double angle_mm = static_cast<double>(state.value.angle_0p001mm) * 0.001;
  const double effort_nm = static_cast<double>(state.value.effort_0p001Nm) * 0.001;
  std::cout << prefix << "位置=" << std::fixed << std::setprecision(2) << angle_mm
            << "mm 力矩=" << std::setprecision(3) << effort_nm
            << "N·m status=0x" << std::hex << static_cast<int>(state.value.status_code) << std::dec
            << " hz=" << std::setprecision(1) << state.value.hz << "\n";
}

}  // namespace

int main() {
  std::signal(SIGINT, OnSignal);

  piper::gripper::Config cfg;
  cfg.can_ifname = "can1";

  piper::gripper::GripperClient client(cfg);

  std::cout << "[INFO] 启动 C++ 夹爪客户端, CAN=" << cfg.can_ifname << "\n";
  auto start_ret = client.Start();
  if (start_ret.code != piper::gripper::ErrorCode::Ok) {
    std::cerr << "[ERROR] Start 失败: " << start_ret.message << "\n";
    return 1;
  }

  std::cout << "[INFO] 注意: 本示例仅调用夹爪C++接口, 请确保机械臂已在可控状态(主从/使能已完成)。\n";

  std::cout << "[INFO] 夹爪测试: 打开(沿用原脚本参数 angle=0, effort=1000)...\n";
  piper::gripper::GripperCommand cmd_open;
  cmd_open.angle_0p001mm = 0;
  cmd_open.effort_0p001Nm = 1000;
  cmd_open.status = piper::gripper::StatusCode::Enable;
  cmd_open.set_zero = piper::gripper::SetZero::Invalid;
  auto send_ret = client.GripperCtrl(cmd_open);
  if (send_ret.code != piper::gripper::ErrorCode::Ok) {
    std::cerr << "[ERROR] GripperCtrl(打开)失败: " << send_ret.message << "\n";
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  PrintState(client.GetArmGripperMsgs(), "  [打开后] ");

  std::cout << "[INFO] 夹爪测试: 关闭(沿用原脚本参数 angle=50000, effort=1000)...\n";
  piper::gripper::GripperCommand cmd_close;
  cmd_close.angle_0p001mm = 50000;
  cmd_close.effort_0p001Nm = 1000;
  cmd_close.status = piper::gripper::StatusCode::Enable;
  cmd_close.set_zero = piper::gripper::SetZero::Invalid;
  send_ret = client.GripperCtrl(cmd_close);
  if (send_ret.code != piper::gripper::ErrorCode::Ok) {
    std::cerr << "[ERROR] GripperCtrl(关闭)失败: " << send_ret.message << "\n";
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  PrintState(client.GetArmGripperMsgs(), "  [关闭后] ");

  std::cout << "[INFO] 持续打印夹爪状态 (Ctrl+C 退出)...\n";
  while (!g_stop.load()) {
    auto state = client.GetArmGripperMsgs();
    if (state.code == piper::gripper::ErrorCode::Ok) {
      const double angle_mm = static_cast<double>(state.value.angle_0p001mm) * 0.001;
      const double effort_nm = static_cast<double>(state.value.effort_0p001Nm) * 0.001;
      std::cout << "\r  Gripper: 位置=" << std::fixed << std::setprecision(2) << angle_mm
                << "mm 力矩=" << std::setprecision(3) << effort_nm
                << "N·m status=0x" << std::hex << static_cast<int>(state.value.status_code) << std::dec
                << " hz=" << std::setprecision(1) << state.value.hz
                << " isOk=" << (client.IsOk() ? "true " : "false") << std::flush;
    } else {
      std::cout << "\r  Gripper: 读取失败: " << state.message << "      " << std::flush;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::cout << "\n[INFO] 退出。\n";

  (void)client.Stop();
  return 0;
}

