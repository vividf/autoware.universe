// Copyright 2024 Tier IV, Inc.
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

#include "pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_nodelet.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
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
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "concatenate_data_synchronizer");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  //  initialize parameters
  params_.output_frame = static_cast<std::string>(declare_parameter("output_frame", ""));
  if (params_.output_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "Need an 'output_frame' parameter to be set before continuing!");
    return;
  }
  params_.input_topics = declare_parameter("input_topics", std::vector<std::string>());
  params_.input_topics = get_parameter("input_topics").as_string_array();
  if (params_.input_topics.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing!");
    return;
  }
  if (params_.input_topics.size() == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  params_.input_twist_topic_type =
    declare_parameter<std::string>("input_twist_topic_type", "twist");

  params_.maximum_queue_size = static_cast<int>(declare_parameter("maximum_queue_size", 5));
  params_.timeout_sec = static_cast<double>(declare_parameter("timeout_sec", 0.1));
  params_.sensor_timestamp_threshold =
    static_cast<double>(declare_parameter("sensor_timestamp_threshold", 0.02)); 

  params_.publish_synchronized_pointcloud =
    declare_parameter("publish_synchronized_pointcloud", true);
  params_.keep_input_frame_in_synchronized_pointcloud =
    declare_parameter("keep_input_frame_in_synchronized_pointcloud", true);
  params_.synchronized_pointcloud_postfix =
    declare_parameter("synchronized_pointcloud_postfix", "pointcloud");

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
    std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> callback =
      std::bind(
        &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this,
        std::placeholders::_1, params_.input_topics[topic_id]);

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

  // Diagnostic Updater
  updater_.setHardwareID("concatenate_data_checker");
  updater_.add(
   "concat_status", this, &PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus);

  // Cloud handler
  combine_cloud_handler_ = std::make_shared<CombineCloudHandler>(
    this, params_.input_topics, params_.keep_input_frame_in_synchronized_pointcloud,
    params_.output_frame);
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
                      << " have the same postfix with synchronized pointcloud. We use the postfix "
                         "to the end of the topic name.");
    replaced_topic_name = original_topic_name + DEFAULT_SYNC_TOPIC_POSTFIX;
  }
  return replaced_topic_name;
}

void PointCloudConcatenateDataSynchronizerComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  sensor_msgs::msg::PointCloud2::SharedPtr xyzi_input_ptr(new sensor_msgs::msg::PointCloud2());
  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_ptr);
  if (input->data.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
  } else {
    // convert to XYZI pointcloud if pointcloud is not empty
    combine_cloud_handler_->convertToXYZICloud(input, xyzi_input_ptr);
  }

  // For each callback, check whether there is a exist collecotor that matches this cloud
  for (const auto & cloud_collector : cloud_collectors_) {
    if (
      std::abs(cloud_collector->getTimeStamp() - rclcpp::Time(input_ptr->header.stamp).seconds()) <
      params_.sensor_timestamp_threshold) {
      cloud_collector->processCloud(topic_name, input_ptr);
    } else {
      auto new_cloud_collector = std::make_unique<CloudCollector>(
        std::dynamic_pointer_cast<PointCloudConcatenateDataSynchronizerComponent>(
          shared_from_this()),
        cloud_collectors_, combine_cloud_handler_, params_.input_topics.size());
      cloud_collector->setTimeStamp(rclcpp::Time(input_ptr->header.stamp).seconds());
      cloud_collector->processCloud(topic_name, input_ptr);
      cloud_collectors_.push_back(std::move(new_cloud_collector));
    }
  }
}

void PointCloudConcatenateDataSynchronizerComponent::twist_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input)
{
  combine_cloud_handler_->process_twist(input);
}

void PointCloudConcatenateDataSynchronizerComponent::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input)
{
  combine_cloud_handler_->process_odometry(input);
}

void PointCloudConcatenateDataSynchronizerComponent::publishClouds()
{
  stop_watch_ptr_->toc("processing_time", true);

  missed_cloud_.clear();
  const auto & topic_to_transformed_cloud_map_ =
    combine_cloud_handler_->getTopicToTransformedCloudMap();
  const auto & concat_cloud_ptr = combine_cloud_handler_->getConcatenatePointcloud();

  // TODO(vivid): remember the case when it is null
  auto concat_output = std::make_unique<sensor_msgs::msg::PointCloud2>(*concat_cloud_ptr);
  concatenate_cloud_publisher_->publish(std::move(concat_output));

  // publish transformed raw pointclouds
  for (const auto & pair : topic_to_transformed_cloud_map_) {
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
      missed_cloud_.insert(pair.first);
    }
  }
  

  combine_cloud_handler_.reset();
  updater_.force_update();

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    for (const auto & pair : topic_to_transformed_cloud_map_) {
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

void PointCloudConcatenateDataSynchronizerComponent::checkConcatStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{

  for (const std::string & input_topic : params_.input_topics) {
    const std::string subscribe_status =
      missed_cloud_.count(input_topic) ? "NG" : "OK";
    stat.add(input_topic, subscribe_status);
  }

  const int8_t level = missed_cloud_.empty()
                         ? diagnostic_msgs::msg::DiagnosticStatus::OK
                         : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  const std::string message = missed_cloud_.empty()
                                ? "Concatenate all topics"
                                : "Some topics are not concatenated";
  stat.summary(level, message);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
