#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "LIVMapper.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 全局禁用 stdout 缓冲，所有的 printf 都会立刻输出
  setvbuf(stdout, NULL, _IONBF, 0);
  // 注册信号处理
  signal(SIGINT, LIVMapper::OnSignal);

  auto node = std::make_shared<LIVMapper>();

  node->initializeComponents();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}