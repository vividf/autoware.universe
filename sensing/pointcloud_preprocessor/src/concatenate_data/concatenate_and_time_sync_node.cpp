// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: concatenate_data.cpp 35231 2011-01-14 05:33:20Z rusu $
 *
 */

#include "pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include "pointcloud_preprocessor/utility/memory.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#define DEFAULT_SYNC_TOPIC_POSTFIX \
  "_synchronized"  // default postfix name for synchronized pointcloud

namespace pointcloud_preprocessor
{

PointCloudConcatenateDataSynchronizerComponent::PointCloudConcatenateDataSynchronizerComponent(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_concatenator_component", node_options)
{
  // initialize debug tool
  using autoware::universe_utils::DebugPublisher;
  using autoware::universe_utils::StopWatch;
  stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<DebugPublisher>(this, "concatenate_data_synchronizer");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  //  initialize parameters
  params_.maximum_queue_size = declare_parameter<int>("maximum_queue_size");
  params_.timeout_sec = declare_parameter<double>("timeout_sec");
  params_.is_motion_compensated = declare_parameter<bool>("is_motion_compensated");
  params_.publish_synchronized_pointcloud =
    declare_parameter<bool>("publish_synchronized_pointcloud");
  params_.keep_input_frame_in_synchronized_pointcloud =
    declare_parameter<bool>("keep_input_frame_in_synchronized_pointcloud");
  params_.publish_previous_but_late_pointcloud =
    declare_parameter<bool>("publish_previous_but_late_pointcloud");
  params_.synchronized_pointcloud_postfix =
    declare_parameter<std::string>("synchronized_pointcloud_postfix");
  params_.input_twist_topic_type = declare_parameter<std::string>("input_twist_topic_type");
  params_.input_topics = declare_parameter<std::vector<std::string>>("input_topics");
  params_.output_frame = declare_parameter<std::string>("output_frame");
  params_.lidar_timestamp_offsets =
    declare_parameter<std::vector<double>>("lidar_timestamp_offsets");
  params_.lidar_timestamp_noise_window =
    declare_parameter<std::vector<double>>("lidar_timestamp_noise_window");

  if (params_.input_topics.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
    return;
  } else if (params_.input_topics.size() == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  if (params_.output_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
    return;
  }
  if (params_.lidar_timestamp_offsets.size() != params_.input_topics.size()) {
    RCLCPP_ERROR(
      get_logger(), "The number of topics does not match the number of timestamp offsets");
    return;
  }
  if (params_.lidar_timestamp_noise_window.size() != params_.input_topics.size()) {
    RCLCPP_ERROR(
      get_logger(), "The number of topics does not match the number of timestamp noise windwo");
    return;
  }

  for (size_t i = 0; i < params_.input_topics.size(); i++) {
    topic_to_offset_map_[params_.input_topics[i]] = params_.lidar_timestamp_offsets[i];
    topic_to_noise_window_map[params_.input_topics[i]] = params_.lidar_timestamp_noise_window[i];
  }

  // Publishers
  concatenate_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));

  // Transformed Raw PointCloud2 Publisher to publish the transformed pointcloud
  if (params_.publish_synchronized_pointcloud) {
    for (auto & topic : params_.input_topics) {
      std::string new_topic =
        replaceSyncTopicNamePostfix(topic, params_.synchronized_pointcloud_postfix);
      auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        new_topic, rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
      topic_to_transformed_cloud_publisher_map_.insert({topic, publisher});
    }
  }

  // Subscribers
  if (params_.input_twist_topic_type == "twist") {
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "~/input/twist", rclcpp::QoS{100},
      std::bind(
        &PointCloudConcatenateDataSynchronizerComponent::twist_callback, this,
        std::placeholders::_1));
  } else if (params_.input_twist_topic_type == "odom") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odom", rclcpp::QoS{100},
      std::bind(
        &PointCloudConcatenateDataSynchronizerComponent::odom_callback, this,
        std::placeholders::_1));
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(), "input_twist_topic_type is invalid: " << params_.input_twist_topic_type);
    throw std::runtime_error(
      "input_twist_topic_type is invalid: " + params_.input_twist_topic_type);
  }

  pointcloud_subs.resize(params_.input_topics.size());
  for (size_t topic_id = 0; topic_id < params_.input_topics.size(); ++topic_id) {
    std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> callback = std::bind(
      &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this, std::placeholders::_1,
      params_.input_topics[topic_id]);

    pointcloud_subs[topic_id].reset();
    pointcloud_subs[topic_id] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.input_topics[topic_id], rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size),
      callback);
  }
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Subscribing to " << params_.input_topics.size() << " user given topics as inputs:");
  for (auto & input_topic : params_.input_topics) {
    RCLCPP_DEBUG_STREAM(get_logger(), " - " << input_topic);
  }

  // Cloud handler
  combine_cloud_handler_ = std::make_shared<CombineCloudHandler>(
    this, params_.input_topics, params_.output_frame, params_.is_motion_compensated,
    params_.keep_input_frame_in_synchronized_pointcloud);

  // Diagnostic Updater
  updater_.setHardwareID("concatenate_data_checker");
  updater_.add(
    "concat_status", this, &PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus);
}

std::string PointCloudConcatenateDataSynchronizerComponent::replaceSyncTopicNamePostfix(
  const std::string & original_topic_name, const std::string & postfix)
{
  std::string replaced_topic_name;
  // separate the topic name by '/' and replace the last element with the new postfix
  size_t pos = original_topic_name.find_last_of("/");
  if (pos == std::string::npos) {
    // not found '/': this is not a namespaced topic
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The topic name is not namespaced. The postfix will be added to the end of the topic name.");
    return original_topic_name + postfix;
  } else {
    // replace the last element with the new postfix
    replaced_topic_name = original_topic_name.substr(0, pos) + "/" + postfix;
  }

  // if topic name is the same with original topic name, add postfix to the end of the topic name
  if (replaced_topic_name == original_topic_name) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The topic name "
                      << original_topic_name
                      << " have the same postfix with synchronized pointcloud. We use "
                         "the postfix "
                         "to the end of the topic name.");
    replaced_topic_name = original_topic_name + DEFAULT_SYNC_TOPIC_POSTFIX;
  }
  return replaced_topic_name;
}

void PointCloudConcatenateDataSynchronizerComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, const std::string & topic_name)
{
  if (!utils::is_data_layout_compatible_with_point_xyzirc(*input_ptr)) {
    RCLCPP_ERROR(
      get_logger(), "The pointcloud layout is not compatible with PointXYZIRC. Aborting");

    if (utils::is_data_layout_compatible_with_point_xyzi(*input_ptr)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy code/data");
    }

    return;
  }

  // Get the current time
  // rclcpp::Time now = debug_clock->now();
  // std::cout << "Current time: " << now.seconds() << " seconds" << std::endl;

  // std::cout << "pointcloud name and timestamp: " << topic_name << " " << std::fixed
  //             << std::setprecision(9) << rclcpp::Time(input_ptr->header.stamp).seconds() <<
  //             std::endl;
  sensor_msgs::msg::PointCloud2::SharedPtr xyzirc_input_ptr(new sensor_msgs::msg::PointCloud2());
  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);
  if (input->data.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
  } else {
    // convert to XYZI pointcloud if pointcloud is not empty
    combine_cloud_handler_->convertToXYZIRCCloud(input, xyzirc_input_ptr);
  }

  // protect collect list
  std::unique_lock<std::mutex> lock(mutex_);

  // For each callback, check whether there is a exist collector that matches this cloud
  bool collector_found = false;

  if (!cloud_collectors_.empty()) {
    std::cout << "Searching collect in size:  " << cloud_collectors_.size() << std::endl;
    for (const auto & cloud_collector : cloud_collectors_) {
      auto [reference_timestamp_min, reference_timestamp_max] =
        cloud_collector->getReferenceTimeStampBoundary();

      if (
        rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name] <
          reference_timestamp_max + topic_to_noise_window_map[topic_name] &&
        rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name] >
          reference_timestamp_min - topic_to_noise_window_map[topic_name]) {
        lock.unlock();
        cloud_collector->processCloud(topic_name, input_ptr);
        collector_found = true;
        std::cout << "find collector " << cloud_collector << std::endl;
        break;
      }
    }
  }
  // if collecotrs is empty or didn't find matched collector.
  if (!collector_found) {
    std::cout << "create new collector " << std::endl;
    auto new_cloud_collector = std::make_shared<CloudCollector>(
      std::dynamic_pointer_cast<PointCloudConcatenateDataSynchronizerComponent>(shared_from_this()),
      cloud_collectors_, combine_cloud_handler_, params_.input_topics.size(), params_.timeout_sec);

    cloud_collectors_.push_back(new_cloud_collector);
    lock.unlock();
    new_cloud_collector->setReferenceTimeStamp(
      rclcpp::Time(input_ptr->header.stamp).seconds() - topic_to_offset_map_[topic_name],
      topic_to_noise_window_map[topic_name]);
    new_cloud_collector->processCloud(topic_name, input_ptr);
  }
}

void PointCloudConcatenateDataSynchronizerComponent::twist_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input)
{
  combine_cloud_handler_->processTwist(input);
}

void PointCloudConcatenateDataSynchronizerComponent::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input)
{
  combine_cloud_handler_->processOdometry(input);
}

void PointCloudConcatenateDataSynchronizerComponent::publishClouds()
{
  stop_watch_ptr_->toc("processing_time", true);

  auto topic_to_transformed_cloud_map = combine_cloud_handler_->getTopicToTransformedCloudMap();
  const auto & concat_cloud_ptr = combine_cloud_handler_->getConcatenatePointcloud();
  std::tie(diagnostic_reference_timestamp_min_, diagnostic_reference_timestamp_max_) =
    combine_cloud_handler_->getReferenceTimeStampBoundary();

  double current_concat_cloud_timestamp = rclcpp::Time(concat_cloud_ptr->header.stamp).seconds();
  if (
    current_concat_cloud_timestamp < lastest_concat_cloud_timestamp_ &&
    !params_.publish_previous_but_late_pointcloud) {
    drop_previous_but_late_pointcloud_ = true;
  } else {
    publish_pointcloud_ = true;
    lastest_concat_cloud_timestamp_ = rclcpp::Time(concat_cloud_ptr->header.stamp).seconds();
    auto concat_output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr);
    concatenate_cloud_publisher_->publish(std::move(concat_output));

    // publish transformed raw pointclouds
    for (const auto & pair : topic_to_transformed_cloud_map) {
      if (pair.second) {
        if (params_.publish_synchronized_pointcloud) {
          auto transformed_cloud_output =
            std::make_unique<sensor_msgs::msg::PointCloud2>(*pair.second);
          topic_to_transformed_cloud_publisher_map_[pair.first]->publish(
            std::move(transformed_cloud_output));
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(), "transformed_raw_points[%s] is nullptr, skipping pointcloud publish.",
          pair.first.c_str());
      }
    }
  }

  updater_.force_update();
  combine_cloud_handler_->resetCloud();

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    for (const auto & pair : topic_to_transformed_cloud_map) {
      if (pair.second != nullptr) {
        const auto pipeline_latency_ms =
          std::chrono::duration<double, std::milli>(
            std::chrono::nanoseconds(
              (this->get_clock()->now() - pair.second->header.stamp).nanoseconds()))
            .count();
        debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
          "debug" + pair.first + "/pipeline_latency_ms", pipeline_latency_ms);
      }
    }
  }
}

std::string PointCloudConcatenateDataSynchronizerComponent::formatTimestamp(double timestamp)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << timestamp;
  return oss.str();
}

void PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (publish_pointcloud_ || drop_previous_but_late_pointcloud_) {
    std::set<std::string> missed_cloud;
    stat.add("reference timestamp min", formatTimestamp(diagnostic_reference_timestamp_min_));
    stat.add("reference timestamp max", formatTimestamp(diagnostic_reference_timestamp_max_));

    auto topic_to_original_stamp_map_ = combine_cloud_handler_->getTopicToOriginalStampMap();

    bool topic_miss = false;
    for (const auto & pair : topic_to_original_stamp_map_) {
      std::string subscribe_status;
      if (pair.second != -1) {
        subscribe_status = "OK " + formatTimestamp(pair.second);
      } else {
        topic_miss = true;
        subscribe_status = "NG";
      }
      stat.add(pair.first, subscribe_status);
    }

    const int8_t level = topic_miss ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                    : diagnostic_msgs::msg::DiagnosticStatus::OK;

    std::string message;
    if (drop_previous_but_late_pointcloud_) {
      message = "Concatenated pointcloud is not published as it is too late";
    } else {
      message = topic_miss ? "Concatenated pointcloud is published but miss some topics"
                           : "Concatenated pointcloud is published and include all topics";
    }
    stat.summary(level, message);

    publish_pointcloud_ = false;
    drop_previous_but_late_pointcloud_ = false;
  } else {
    const int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    const std::string message =
      "Concatenate node launch successfully, but waiting for input pointcloud";
    stat.summary(level, message);
  }
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
