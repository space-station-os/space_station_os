#include "space_station_control/teleop.hpp"
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <csignal>

using space_station_control::msg::ThrustersCmd;

// --- Terminal Manager ---
class TerminalManager {
public:
  TerminalManager() {
    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
  }

  ~TerminalManager() {
    restore();
  }

  void restore() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
  }

private:
  struct termios old_tio_, new_tio_;
};

// --- Global handler ---
TerminalManager* global_terminal_mgr = nullptr;

void signal_handler(int) {
  if (global_terminal_mgr) global_terminal_mgr->restore();
  std::cout << "\n[Exit] Teleop node shut down cleanly.\n";
  rclcpp::shutdown();
  exit(0);
}

// --- Teleop Node Implementation ---
TeleopNode::TeleopNode() : Node("teleop_node") {
  command_pub_ = this->create_publisher<ThrustersCmd>("/thruster_command", 10);
  std::thread input_thread(&TeleopNode::input_loop, this);
  input_thread.detach();

  RCLCPP_INFO(this->get_logger(), "Teleop Node Started. Use keys to control:");
}

void TeleopNode::input_loop() {
  TerminalManager terminal_mgr;
  global_terminal_mgr = &terminal_mgr;
  std::signal(SIGINT, signal_handler);

  std::cout << "\nControls:\n"
            << "  W - Forward\n"
            << "  S - Backward\n"
            << "  A - Up\n"
            << "  D - Down\n"
            << "  Q - Yaw Left\n"
            << "  E - Yaw Right\n"
            << "SPACE - Halt\n"
            << "Ctrl+C to exit.\n\n";

  unsigned char c;
  while (rclcpp::ok()) {
    c = getchar();

    auto msg = ThrustersCmd();
    msg.thrust = 8000.0;

    switch (c) {
      case 'w': case 'W': msg.direction = "forward"; break;
      case 's': case 'S': msg.direction = "backward"; break;
      case 'a': case 'A': msg.direction = "up"; break;
      case 'd': case 'D': msg.direction = "down"; break;
      case 'q': case 'Q': msg.direction = "yaw_left"; break;
      case 'e': case 'E': msg.direction = "yaw_right"; break;
      case ' ': msg.direction = "halt"; msg.thrust = 0.0; break;
      default:
        std::cout << "[Invalid key] Use W/S/A/D/Q/E or SPACE.\n";
        continue;
    }

    command_pub_->publish(msg);
    std::cout << "[Sent] " << msg.direction << " @ " << msg.thrust << " thrust\n";
  }
}

// --- Main ---
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
