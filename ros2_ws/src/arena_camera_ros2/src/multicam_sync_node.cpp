#include <rclcpp/rclcpp.hpp>
#include "MultiCamSync.h"

std::shared_ptr<arena_camera_ros2::MultiCamSync> g_node;
bool g_shutting_down;

void signal_callback_handler(int signum) {
  if (!g_shutting_down) {
    g_shutting_down = true;
    rclcpp::shutdown();
    signal(SIGINT, signal_callback_handler);
    fflush(stdout);
    g_node->shutdown_handler();
  } else {
    signal(SIGINT, signal_callback_handler);
    fflush(stdout);
  }
}

int main(int argc, char** argv) {
  g_shutting_down = false;
  rclcpp::init(argc, argv);
  signal(SIGINT, signal_callback_handler);
  g_node = std::make_shared<arena_camera_ros2::MultiCamSync>();
  rclcpp::spin(g_node);

  return 0;
}