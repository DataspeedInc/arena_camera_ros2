#include "MultiCamSync.h"

namespace arena_camera_ros2 {

  MultiCamSync::MultiCamSync(const rclcpp::NodeOptions& options)
  : rclcpp::Node("multi_cam_sync", options)
  {
    delay_time_ns = (uint64_t)(1e9 / this->declare_parameter<double>("frame_rate", 10.0));
    param_cb = add_on_set_parameters_callback(std::bind(&MultiCamSync::param_update, this, std::placeholders::_1));

    camera_discovery_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MultiCamSync::camera_discovery, this));
    m_pSystem.reset(Arena::OpenSystem());
  }

  MultiCamSync::~MultiCamSync() {
  }

  void MultiCamSync::camera_discovery() {
    RCLCPP_INFO(get_logger(), "Discovering cameras...");
    m_pSystem->UpdateDevices(100);  // in millisec

    // For some reason there are two entries for each camera... remove duplicate serial numbers
    auto raw_device_infos = m_pSystem->GetDevices();
    std::vector<Arena::DeviceInfo> device_infos;
    std::map<std::string, bool> used_serial_numbers;
    for (auto& raw_dev : raw_device_infos) {
      std::string serial_number(raw_dev.SerialNumber().c_str());
      if (used_serial_numbers.find(serial_number) == used_serial_numbers.end()) {
        used_serial_numbers[serial_number] = true;
        device_infos.push_back(raw_dev);
      }
    }

    if (device_infos.empty()) {
      RCLCPP_INFO(get_logger(), "Waiting for a nonzero number of cameras to be detected");
      return;
    }

    for (auto& dev_info : device_infos) {
      std::string serial_number(dev_info.SerialNumber().c_str());
      RCLCPP_INFO_STREAM(get_logger(), "Detected camera with serial number " << dev_info.SerialNumber());
      Arena::IDevice* dev_ptr;
      try {
        dev_ptr = m_pSystem->CreateDevice(dev_info);
        RCLCPP_INFO(get_logger(), "Successfully created device");
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Failed to create device");
        continue;
      }
      cameras.push_back(SingleCamera(
        shared_from_this(),
        &delay_time_ns,
        serial_number,
        dev_ptr)
      );
    }

    camera_discovery_timer->cancel();
    std::thread{std::bind(&MultiCamSync::run, this)}.detach();
  }

  void MultiCamSync::run() {

	  Arena::SetNodeValue<int64_t>(
      m_pSystem->GetTLSystemNodeMap(),
      "ActionCommandDeviceKey",
      1);

    Arena::SetNodeValue<int64_t>(
      m_pSystem->GetTLSystemNodeMap(),
      "ActionCommandGroupKey",
      1);

    Arena::SetNodeValue<int64_t>(
      m_pSystem->GetTLSystemNodeMap(),
      "ActionCommandGroupMask",
      1);

    Arena::SetNodeValue<int64_t>(
      m_pSystem->GetTLSystemNodeMap(),
      "ActionCommandTargetIP",
      0xFFFFFFFF);


    bool ptp_negotiation_complete = false;
    RCLCPP_INFO(get_logger(), "Starting PTP negotiation");
    while (rclcpp::ok()) {
      try {
        if (!ptp_negotiation_complete) {

          bool master_found = false;
          bool try_again = false;

          for (auto& cam : cameras) {
            auto ptp_status = cam.get_ptp_status();
            if (ptp_status == "Master") {
              if (master_found) {
                // Multiple masters... not done yet
                try_again = true;
                break;
              }

              master_found = true;
            } else if (ptp_status != "Slave") {
              try_again = true;
              break;
            } else {
            }
          }

          ptp_negotiation_complete = (!try_again && master_found);

          if (ptp_negotiation_complete) {
            RCLCPP_INFO(get_logger(), "PTP negotiation complete! Starting to stream images");
            break;
          } else {
            std::this_thread::sleep_for(std::chrono::duration<int>(1));
          }
        }
      } catch (std::exception& ex) {
        RCLCPP_ERROR_STREAM(get_logger(), "Caught Exception:\n" << ex.what());
      }
    }

    RCLCPP_INFO(get_logger(), "Begin starting streams");
    for (auto& cam : cameras) {
      try {
        cam.get_device_ptr()->StartStream();
      } catch (GenICam_3_3_LUCID::LogicalErrorException& ex) {
        RCLCPP_WARN_STREAM(get_logger(), "Logical error exception:\n" << ex.what());
      }
    }
    RCLCPP_INFO(get_logger(), "End starting streams");
    uint64_t last_image_stamp = 0;

    while (rclcpp::ok()) {
      try {
        for (size_t i = 0; i < cameras.size(); i++) {
          if (i == 0) {
            if (last_image_stamp) {
              Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(), "ActionCommandExecuteTime", (int64_t)last_image_stamp + delay_time_ns);
            } else {
              Arena::ExecuteNode(cameras[0].get_device_ptr()->GetNodeMap(), "PtpDataSetLatch");
              int64_t curr_ptp = Arena::GetNodeValue<int64_t>(cameras[0].get_device_ptr()->GetNodeMap(), "PtpDataSetLatchValue");
              Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(), "ActionCommandExecuteTime", curr_ptp + delay_time_ns);
            }
            Arena::ExecuteNode(m_pSystem->GetTLSystemNodeMap(), "ActionCommandFireCommand");
          }
          last_image_stamp = cameras[i].publish_image();
        }
      } catch (GenICam_3_3_LUCID::AccessException& ex) {
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 500, "Acess exception during stream:\n" << ex.what());
      }
    }
  }

  void MultiCamSync::shutdown_handler() {
    if (m_pSystem) {
      for (auto& cam : cameras) {
        try {
          auto dev_ptr = cam.get_device_ptr();
          if (dev_ptr) {
            std::cout << "Shutting down camera " << cam.get_device_serial_number() << "\n  Stopping stream\n";
            dev_ptr->StopStream();
            sleep(2);
            std::cout << "  Cleaning up camera interface memory\n";
            m_pSystem->DestroyDevice(dev_ptr);
          }
        } catch (GenICam_3_3_LUCID::LogicalErrorException& ex) {
          std::cout << "Logical error exception during shutdown:\n" << ex.what() << std::endl;
          sleep(2);
        } catch (GenICam_3_3_LUCID::InvalidArgumentException& ex) {
          std::cout << "Invalid argument exception during shutdown:\n" << ex.what() << std::endl;
          sleep(2);
        }
      }
      Arena::CloseSystem(m_pSystem.get());
    }
  }

  rcl_interfaces::msg::SetParametersResult MultiCamSync::param_update(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    for (const rclcpp::Parameter& p : parameters) {
      if (p.get_name() == "frame_rate") {
        auto new_frame_rate = std::clamp(p.as_double(), 1.0, 30.0);
        RCLCPP_INFO_STREAM(get_logger(), "Syncronizing frame capture at " << new_frame_rate << " fps");
        delay_time_ns = (uint64_t)(1e9 / new_frame_rate);
      } else {
        auto full_name = p.get_name();

        size_t period_loc = full_name.find('.');
        size_t underscore_loc = full_name.find('_');
        if (period_loc != std::string::npos && underscore_loc != std::string::npos && period_loc > underscore_loc) {
          std::string parameter_namespace = full_name.substr(underscore_loc + 1, period_loc - underscore_loc - 1);
          std::string parameter_name = full_name.substr(period_loc + 1);
          for (auto& cam : cameras) {
            if (cam.get_device_serial_number() == parameter_namespace) {
              if (parameter_name == "gain") {
                cam.set_gain(p.as_double());
              } else if (parameter_name == "exposure") {
                cam.set_exposure(p.as_double());
              } else if (parameter_name == "gamma") {
                cam.set_gamma(p.as_double());
              }
              break;
            }
          }
        }
      }
    }

    result.successful = true;
    return result;
  }

}
