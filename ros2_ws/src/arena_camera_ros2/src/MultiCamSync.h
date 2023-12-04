#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "SingleCamera.h"

namespace arena_camera_ros2 {

  class MultiCamSync : public rclcpp::Node {
    public:
      MultiCamSync(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
      ~MultiCamSync();

      void shutdown_handler();

    private:
      void run();
      void camera_discovery();

      rclcpp::TimerBase::SharedPtr camera_discovery_timer;

      std::vector<SingleCamera> cameras;
      std::shared_ptr<Arena::ISystem> m_pSystem;

      // ROS parameters
      uint64_t delay_time_ns;
      double gain;
      double exposure_time;
      double gamma;

      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb;
      rcl_interfaces::msg::SetParametersResult param_update(const std::vector<rclcpp::Parameter>& parameters);
  };

}