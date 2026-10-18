#include "nezha_lqr_controller/lqrController.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lqrController>();
  
  // 设置实时调度
  struct sched_param sched_params;
  sched_params.sched_priority = 90; // 设置高优先级
  if (sched_setscheduler(0, SCHED_FIFO, &sched_params) == -1) {
    RCLCPP_WARN(node->get_logger(), "Failed to set real-time priority: %s", strerror(errno));
  } else {
    RCLCPP_INFO(node->get_logger(), "Real-time scheduling enabled with priority %d", sched_params.sched_priority);
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

