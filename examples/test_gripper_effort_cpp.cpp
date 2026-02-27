#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "piper/gripper/gripper_client.hpp"

namespace {

void PrintGripper(const piper::gripper::Result<piper::gripper::GripperState>& state,
                  const char* label) {
  if (state.code != piper::gripper::ErrorCode::Ok) {
    std::cout << "  [" << label << "] 读取失败: " << state.message << "\n";
    return;
  }

  const double angle_mm = static_cast<double>(state.value.angle_0p001mm) * 0.001;
  const double effort_nm = static_cast<double>(state.value.effort_0p001Nm) * 0.001;
  std::cout << "  [" << label << "] 位置=" << std::fixed << std::setprecision(2) << angle_mm
            << "mm 实际力矩=" << std::setprecision(3) << effort_nm << "N·m\n";
}

}  // namespace

int main() {
  constexpr const char* kCanName = "can0";
  const std::vector<uint16_t> kEffortLevels = {200, 500, 1000};
  constexpr int32_t kCloseTarget = 0;
  constexpr int32_t kOpenTarget = 60000;
  constexpr double kHoldTimeSec = 3.0;

  piper::gripper::Config cfg;
  cfg.can_ifname = kCanName;

  piper::gripper::GripperClient client(cfg);

  std::cout << "[INFO] 连接 " << kCanName << " ...\n";
  auto start_ret = client.Start();
  if (start_ret.code != piper::gripper::ErrorCode::Ok) {
    std::cerr << "[ERROR] Start失败: " << start_ret.message << "\n";
    return 1;
  }

  std::cout << "[INFO] 注意: 本示例仅调用夹爪C++接口, 请先确保机械臂处于可控状态。\n";

  std::cout << "[INFO] 初始化: 打开夹爪...\n";
  piper::gripper::GripperCommand open_cmd;
  open_cmd.angle_0p001mm = kOpenTarget;
  open_cmd.effort_0p001Nm = 1000;
  open_cmd.status = piper::gripper::StatusCode::Enable;
  open_cmd.set_zero = piper::gripper::SetZero::Invalid;

  auto ret = client.GripperCtrl(open_cmd);
  if (ret.code != piper::gripper::ErrorCode::Ok) {
    std::cerr << "[ERROR] 打开夹爪失败: " << ret.message << "\n";
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  PrintGripper(client.GetArmGripperMsgs(), "打开后");

  std::cout << "\n=======================================================\n";
  std::cout << "  力矩限幅测试: 夹爪尝试关闭, 使用不同 effort 上限\n";
  std::cout << "  请把物体放入夹爪, 观察不同力矩下的夹持效果\n";
  std::cout << "=======================================================\n\n";
  std::cout << "  按 Enter 开始测试...";
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  for (uint16_t effort : kEffortLevels) {
    const double effort_nm = static_cast<double>(effort) * 0.001;
    std::cout << "\n[TEST] 力矩上限 = " << effort << " (" << std::fixed << std::setprecision(1)
              << effort_nm << " N·m)\n";

    piper::gripper::GripperCommand close_cmd;
    close_cmd.angle_0p001mm = kCloseTarget;
    close_cmd.effort_0p001Nm = effort;
    close_cmd.status = piper::gripper::StatusCode::Enable;
    close_cmd.set_zero = piper::gripper::SetZero::Invalid;

    ret = client.GripperCtrl(close_cmd);
    if (ret.code != piper::gripper::ErrorCode::Ok) {
      std::cerr << "[ERROR] 关闭命令发送失败: " << ret.message << "\n";
    }

    const auto start = std::chrono::steady_clock::now();
    while (true) {
      auto state = client.GetArmGripperMsgs();
      if (state.code == piper::gripper::ErrorCode::Ok) {
        const double angle_mm = static_cast<double>(state.value.angle_0p001mm) * 0.001;
        const double effort_actual = static_cast<double>(state.value.effort_0p001Nm) * 0.001;
        std::cout << "\r    位置=" << std::fixed << std::setw(6) << std::setprecision(2) << angle_mm
                  << "mm  实际力矩=" << std::setprecision(3) << effort_actual << "N·m" << std::flush;
      }

      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed >= kHoldTimeSec) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "\n";

    std::cout << "       打开夹爪...\n";
    ret = client.GripperCtrl(open_cmd);
    if (ret.code != piper::gripper::ErrorCode::Ok) {
      std::cerr << "[ERROR] 打开命令发送失败: " << ret.message << "\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    PrintGripper(client.GetArmGripperMsgs(), "打开后");
  }

  std::cout << "\n[INFO] 测试完成!\n";
  std::cout << "\n[说明]\n";
  std::cout << "  - 有物体时: 小 effort 会在接触后提前停住(柔顺夹持)\n";
  std::cout << "  - 空载时: 夹爪通常会运动到目标位置\n";
  std::cout << "  - effort=1000 (1N·m) 常见抓取较平衡\n";
  std::cout << "  - 建议最大安全值不超过 3000 (3N·m)\n";

  (void)client.Stop();
  return 0;
}

