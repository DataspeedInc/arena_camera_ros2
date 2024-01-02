#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <yaml-cpp/yaml.h>
#include <cv_bridge/cv_bridge.h>

#include "ArenaApi.h"
#include "rclcpp_adapter/pixelformat_translation.h"
namespace arena_camera_ros2 {

class SingleCamera {
  public:
    SingleCamera(rclcpp::Node::SharedPtr node, const uint64_t* delay_time_ns, const std::string& serial_number, Arena::IDevice* dev_ptr);
    ~SingleCamera();

    double gain_lower_limit;
    double gain_upper_limit;

    uint64_t publish_image();
    Arena::IDevice* get_device_ptr();
    std::string get_device_serial_number();
    std::string get_ptp_status();

    void set_gain(double gain);
    void set_exposure(double exposure);
    void set_gamma(double gamma);

  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_img;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info;

    std::shared_ptr<Arena::IDevice> m_pDevice;
    rclcpp::Node::SharedPtr n;

    bool camera_info_available;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    std::string frame_id;
    int64_t width;
    int64_t height;
    std::string pixelformat_ros;
    GenApi_3_3_LUCID::INodeMap* nodemap;
    GenApi_3_3_LUCID::INodeMap* tlstream_nodemap;
    const uint64_t* delay_time_ns;

};

}