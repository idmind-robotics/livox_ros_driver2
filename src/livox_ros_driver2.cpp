//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <algorithm>
#include <stdexcept>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  LddcConfig cfg;

  // Parameters the vendor driver already had. Defaults are unchanged.
  this->declare_parameter("xfer_format", cfg.transfer_format);
  this->declare_parameter("multi_topic", cfg.multi_topic);
  this->declare_parameter("data_src", cfg.data_src);
  this->declare_parameter("publish_freq", cfg.publish_freq);
  this->declare_parameter("output_data_type", cfg.output_type);
  this->declare_parameter("frame_id", cfg.frame_id);
  this->declare_parameter("user_config_path", "path_default");

  // Added here. Every default reproduces the previous hardcoded behaviour.
  this->declare_parameter("imu_frame_id", cfg.imu_frame_id);
  this->declare_parameter("imu_publish_freq", cfg.imu_publish_freq);
  this->declare_parameter("lidar_topic", cfg.lidar_topic);
  this->declare_parameter("imu_topic", cfg.imu_topic);
  this->declare_parameter("qos_reliability", std::string("reliable"));
  this->declare_parameter("qos_depth", cfg.qos_depth);
  this->declare_parameter("timestamp_source", std::string("lidar"));

  this->get_parameter("xfer_format", cfg.transfer_format);
  this->get_parameter("multi_topic", cfg.multi_topic);
  this->get_parameter("data_src", cfg.data_src);
  this->get_parameter("publish_freq", cfg.publish_freq);
  this->get_parameter("output_data_type", cfg.output_type);
  this->get_parameter("frame_id", cfg.frame_id);
  this->get_parameter("imu_frame_id", cfg.imu_frame_id);
  this->get_parameter("imu_publish_freq", cfg.imu_publish_freq);
  this->get_parameter("lidar_topic", cfg.lidar_topic);
  this->get_parameter("imu_topic", cfg.imu_topic);
  this->get_parameter("qos_depth", cfg.qos_depth);

  const std::string reliability = this->get_parameter("qos_reliability").as_string();
  if (reliability == "best_effort") {
    cfg.qos_best_effort = true;
  } else if (reliability != "reliable") {
    DRIVER_FATAL(*this, "qos_reliability must be 'reliable' or 'best_effort', got '%s'",
        reliability.c_str());
    throw std::invalid_argument("invalid qos_reliability");
  }

  const std::string stamp_source = this->get_parameter("timestamp_source").as_string();
  if (stamp_source == "ros") {
    cfg.stamp_with_ros_time = true;
  } else if (stamp_source != "lidar") {
    DRIVER_FATAL(*this, "timestamp_source must be 'lidar' or 'ros', got '%s'",
        stamp_source.c_str());
    throw std::invalid_argument("invalid timestamp_source");
  }

  // These used to be accepted and then quietly do nothing, leaving a node that
  // runs and publishes no data at all. Refuse them instead.
  if (cfg.transfer_format == kPclPxyziMsg) {
    DRIVER_FATAL(*this, "xfer_format=2 (pcl::PointCloud) is a ROS1-only format; "
        "use 0 for PointCloud2 or 1 for the Livox custom message");
    throw std::invalid_argument("unsupported xfer_format");
  }
  if (cfg.transfer_format != kPointCloud2Msg && cfg.transfer_format != kLivoxCustomMsg) {
    DRIVER_FATAL(*this, "xfer_format must be 0 (PointCloud2) or 1 (CustomMsg), got %d",
        cfg.transfer_format);
    throw std::invalid_argument("unsupported xfer_format");
  }
  if (cfg.output_type != kOutputToRos) {
    DRIVER_FATAL(*this, "output_data_type=%d (rosbag) was ROS1-only; record with "
        "'ros2 bag record' instead", cfg.output_type);
    throw std::invalid_argument("unsupported output_data_type");
  }
  if (cfg.data_src != kSourceRawLidar) {
    DRIVER_FATAL(*this, "data_src=%d is not supported; only 0 (raw lidar) works",
        cfg.data_src);
    throw std::invalid_argument("unsupported data_src");
  }

  // Warn on clamping rather than silently moving the rate the user asked for.
  if (cfg.publish_freq > 100.0 || cfg.publish_freq < 0.5) {
    const double requested = cfg.publish_freq;
    cfg.publish_freq = std::min(100.0, std::max(0.5, cfg.publish_freq));
    DRIVER_WARN(*this, "publish_freq %.3f Hz is out of range, clamped to %.3f Hz",
        requested, cfg.publish_freq);
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(cfg);
  lddc_ptr_->SetRosNode(this);

  DRIVER_INFO(*this, "Data Source is raw lidar.");

  std::string user_config_path;
  this->get_parameter("user_config_path", user_config_path);
  DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

  LdsLidar *read_lidar = LdsLidar::GetInstance(cfg.publish_freq);
  lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

  if ((read_lidar->InitLdsLidar(user_config_path))) {
    DRIVER_INFO(*this, "Init lds lidar success!");
  } else {
    DRIVER_ERROR(*this, "Init lds lidar fail!");
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)
