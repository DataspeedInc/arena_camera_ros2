#include "SingleCamera.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace arena_camera_ros2 {

  SingleCamera::SingleCamera(rclcpp::Node::SharedPtr node, const uint64_t* delay_time_ns, const std::string& serial_number, Arena::IDevice* dev_ptr)
  : n(node), delay_time_ns(delay_time_ns)
  {
    if (!dev_ptr) {
      RCLCPP_FATAL_STREAM(n->get_logger(), "Failed to connect to " << serial_number);
      return;
    }

    std::string param_namespace = "camera_" + serial_number;

    std::string camera_info_pkg = n->declare_parameter<std::string>(param_namespace + ".camera_info_package", "");
    std::string camera_info_file = n->declare_parameter<std::string>(param_namespace + ".camera_info_file", "");
    std::string camera_info_path = ament_index_cpp::get_package_share_directory(camera_info_pkg) + "/" + camera_info_file;

    std::string camera_name = n->declare_parameter<std::string>(param_namespace + ".camera_name", param_namespace);
    double gain = n->declare_parameter<double>(param_namespace + ".gain", 10.0);
    gain_lower_limit = n->declare_parameter<double>(param_namespace + ".gain_lower_limit", 1.0);
    gain_upper_limit = n->declare_parameter<double>(param_namespace + ".gain_upper_limit", 48.0);
    double exposure = n->declare_parameter<double>(param_namespace + ".exposure", 2000.0);
    double gamma = n->declare_parameter<double>(param_namespace + ".gamma", 0.5);

    frame_id = camera_name;

    m_pDevice.reset(dev_ptr);
    rclcpp::SensorDataQoS pub_qos_;
    pub_qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    pub_img = n->create_publisher<sensor_msgs::msg::Image>(camera_name + "/image_raw", pub_qos_);

    YAML::Node camera_info_data;
    if (camera_info_file.empty() || camera_info_pkg.empty()) {
      camera_info_available = false;
      RCLCPP_ERROR(n->get_logger(), "No camera info file path provided; no sensor_msgs/CameraInfo messages will be published");
    } else {
      try {
        camera_info_data = YAML::LoadFile(camera_info_path);
        camera_info_msg.width = camera_info_data["image_width"].as<int>();
        camera_info_msg.height = camera_info_data["image_height"].as<int>();
        camera_info_msg.distortion_model = camera_info_data["distortion_model"].as<std::string>();
        auto camera_matrix_data = camera_info_data["camera_matrix"]["data"];
        for (int i = 0; i < 9; i++) {
          camera_info_msg.k[i] = camera_matrix_data[i].as<double>();
        }
        auto distortion_data = camera_info_data["distortion_coefficients"]["data"];
        camera_info_msg.d.reserve(5);
        for (int i = 0; i < 5; i++) {
          camera_info_msg.d[i] = distortion_data[i].as<double>();
        }
        auto rectification_data = camera_info_data["rectification_matrix"]["data"];
        for (int i = 0; i < 9; i++) {
          camera_info_msg.r[i] = rectification_data[i].as<double>();
        }
        auto projection_data = camera_info_data["projection_matrix"]["data"];
        for (int i = 0; i < 12; i++) {
          camera_info_msg.p[i] = projection_data[i].as<double>();
        }
        camera_info_available = true;
      } catch (YAML::BadFile& ex) {
        RCLCPP_ERROR_STREAM(n->get_logger(), "Error parsing camera info file: " << ex.what());
        camera_info_available = false;
      }
    }

    if (camera_info_available) {
      pub_cam_info = n->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/camera_info", pub_qos_);
    }

    try {
      nodemap = m_pDevice->GetNodeMap();
      tlstream_nodemap = m_pDevice->GetTLStreamNodeMap();
      set_exposure(exposure);
      set_gain(gain);
      set_gamma(gamma);

      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  TriggerMode: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode"));
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  TriggerSelector: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector"));
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Action0");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  TriggerSource: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource"));
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ActionUnconditionalMode", "On");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  ActionUnconditionalMode: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "ActionUnconditionalMode"));
      Arena::SetNodeValue<int64_t>(nodemap, "ActionSelector", 0);
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  ActionSelector: " << Arena::GetNodeValue<int64_t>(nodemap, "ActionSelector"));
      Arena::SetNodeValue<int64_t>(nodemap, "ActionDeviceKey", 1);
      Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupKey", 1);
      Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupMask", 1);
      Arena::SetNodeValue<bool>(nodemap, "PtpEnable", true);
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  PtpEnable: " << Arena::GetNodeValue<bool>(nodemap, "PtpEnable"));

      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TransferControlMode", "UserControlled");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  TransferControlMode: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TransferControlMode"));
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TransferOperationMode", "Continuous");
      RCLCPP_DEBUG_STREAM(n->get_logger(), "  TransferOperationMode: " << Arena::GetNodeValue<GenICam::gcstring>(nodemap, "TransferOperationMode"));
      Arena::ExecuteNode(nodemap, "TransferStop");

      width = Arena::GetNodeValue<int64_t>(nodemap, "Width");
      height = Arena::GetNodeValue<int64_t>(nodemap, "Height");
      pixelformat_ros = K_PFNC_TO_ROS2_PIXELFORMAT[Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat").c_str()];
    } catch (GenICam_3_3_LUCID::AccessException& ex) {
      RCLCPP_WARN_STREAM(n->get_logger(), "Access exception during camera initialization:\n" << ex.what());
    } catch (GenICam_3_3_LUCID::RuntimeException& ex) {
      RCLCPP_WARN_STREAM(n->get_logger(), "Runtime exception during camera initialization:\n" << ex.what());
    }
  }

  SingleCamera::~SingleCamera() {
  }

  Arena::IDevice* SingleCamera::get_device_ptr() {
    return m_pDevice.get();
  }

  std::string SingleCamera::get_device_serial_number() {
    return Arena::GetNodeValue<GenICam::gcstring>(nodemap, "DeviceSerialNumber").c_str();
  }

  std::string SingleCamera::get_ptp_status() {
    return Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus").c_str();
  }

  uint64_t SingleCamera::publish_image() {
    Arena::IImage* pImage;
    std::string deviceSerialNumber(Arena::GetNodeValue<GenICam::gcstring>(nodemap, "DeviceSerialNumber").c_str());
    try {
      sensor_msgs::msg::Image image_msg;
      auto then_ns = n->get_clock()->now().nanoseconds();
      Arena::ExecuteNode(nodemap, "TransferStart");
      pImage = m_pDevice->GetImage(2 * (*delay_time_ns) / 1000000);
      Arena::ExecuteNode(nodemap, "TransferStop");

      auto now_ns = n->get_clock()->now().nanoseconds();
      // RCLCPP_INFO_STREAM(n->get_logger(), "Time: " << (now_ns - then_ns) * 1e-6 << " ms");
      image_msg.header.stamp.sec = static_cast<uint32_t>(now_ns / 1000000000);
      image_msg.header.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000);
      image_msg.header.frame_id = frame_id;
      image_msg.height = height;
      image_msg.width = width;
      image_msg.encoding = pixelformat_ros;

      auto image_stamp = pImage->GetTimestamp();

      // RCLCPP_INFO_STREAM(n->get_logger(), "Camera " << deviceSerialNumber << "   Timestamp: " << 1e-9 * pImage->GetTimestamp());

      //
      // 5 ) is_big_endian
      //
      // TODO what to do if unknown
      image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                              Arena::EPixelEndianness::PixelEndiannessBig;
      //
      // 6 ) step
      //
      // TODO could be optimized by moving it out
      auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
      auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
      image_msg.step =
          static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

      //
      // 7) data
      //
      auto image_data_length_in_bytes = width_length_in_bytes * height;
      image_msg.data.resize(image_data_length_in_bytes);
      auto x = pImage->GetData();
      std::memcpy(&image_msg.data[0], pImage->GetData(),
                  image_data_length_in_bytes);

      if (pub_cam_info) {
        camera_info_msg.header = image_msg.header;
        pub_cam_info->publish(camera_info_msg);
      }
      pub_img->publish(std::move(image_msg));
      m_pDevice->RequeueBuffer(pImage);
      return image_stamp;
    } catch (GenICam_3_3_LUCID::RuntimeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), "Runtime exception on camera  " << deviceSerialNumber << ":\n" << ex.what());
    } catch (GenICam_3_3_LUCID::TimeoutException& ex) {
      RCLCPP_DEBUG_STREAM(n->get_logger(), "Timeout on camera  " << deviceSerialNumber << ":\n" << ex.what());
    } catch (GenICam_3_3_LUCID::InvalidArgumentException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), "Invalid argument on camera  " << deviceSerialNumber << ":\n" << ex.what());
    }
    return 0;
  }

  void SingleCamera::set_gain(double gain) {
    try {
      if (gain <= 0) {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Continuous");
        Arena::SetNodeValue<double>(nodemap, "GainAutoLowerLimit", gain_lower_limit);
        Arena::SetNodeValue<double>(nodemap, "GainAutoUpperLimit", gain_upper_limit);
        RCLCPP_INFO_STREAM(n->get_logger(), "Set automatic gain between " << gain_lower_limit << " and " << gain_upper_limit);
      } else {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Off");
        Arena::SetNodeValue<double>(nodemap, "Gain", gain);
        RCLCPP_INFO_STREAM(n->get_logger(), "Set fixed gain to " << gain);
      }
    } catch (GenICam_3_3_LUCID::OutOfRangeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::RuntimeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::LogicalErrorException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    }
  }

  void SingleCamera::set_exposure(double exposure) {
    try {
      if (exposure <= 0) {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoLimitAuto", "Continuous");
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoAlgorithm", "Mean");
        RCLCPP_INFO_STREAM(n->get_logger(), "Set automatic exposure");
      } else {
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
        Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAutoLimitAuto", "Off");
        Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure);
        RCLCPP_INFO_STREAM(n->get_logger(), "Set fixed exposure time to " << exposure);
      }
    } catch (GenICam_3_3_LUCID::OutOfRangeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::RuntimeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::LogicalErrorException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    }
  }

  void SingleCamera::set_gamma(double gamma) {
    try {
      if (gamma <= 0) {
        Arena::SetNodeValue<bool>(nodemap, "GammaEnable", false);
        RCLCPP_INFO_STREAM(n->get_logger(), "Disabled gamma");
      } else {
        Arena::SetNodeValue<bool>(nodemap, "GammaEnable", true);
        Arena::SetNodeValue<double>(nodemap, "Gamma", gamma);
        RCLCPP_INFO_STREAM(n->get_logger(), "Set gamma to " << gamma);
      }
    } catch (GenICam_3_3_LUCID::OutOfRangeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::RuntimeException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    } catch (GenICam_3_3_LUCID::LogicalErrorException& ex) {
      RCLCPP_ERROR_STREAM(n->get_logger(), ex.what());
    }
  }

}