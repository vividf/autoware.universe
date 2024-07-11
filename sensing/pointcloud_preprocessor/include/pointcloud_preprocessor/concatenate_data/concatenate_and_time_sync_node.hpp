// Copyright 2023 TIER IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODE_HPP_
#define POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODE_HPP_

#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware_point_types/types.hpp"
#include "cloud_collector.hpp"
#include "combine_cloud_handler.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

namespace pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent : public rclcpp::Node
{
public:
  explicit PointCloudConcatenateDataSynchronizerComponent(const rclcpp::NodeOptions & node_options);
  virtual ~PointCloudConcatenateDataSynchronizerComponent() {}
  void publishClouds();

private:
  struct Parameters
  {
    int maximum_queue_size;
    double timeout_sec;
    bool is_motion_compensated;
    bool publish_synchronized_pointcloud;
    bool keep_input_frame_in_synchronized_pointcloud;
    std::string synchronized_pointcloud_postfix;
    std::string input_twist_topic_type;
    std::vector<std::string> input_topics;
    std::string output_frame;
    std::vector<double> lidar_timestamp_offsets;
    std::vector<double> lidar_timestamp_noise_window;
  } params_;

  // remove this later
  // rclcpp::Clock::SharedPtr debug_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  std::set<std::string> missed_cloud_;
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  std::shared_ptr<CloudCollector> cloud_collector_;
  std::list<std::shared_ptr<CloudCollector>> cloud_collectors_;
  std::mutex mutex_;
  std::unordered_map<std::string, double> topic_to_offset_map_;
  std::unordered_map<std::string, double> topic_to_noise_window_map;

  // subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr concatenate_cloud_publisher_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
    topic_to_transformed_cloud_publisher_map_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;

  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  diagnostic_updater::Updater updater_{this};

  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr, const std::string & topic_name);
  void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr input);
  void checkConcatStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::string replaceSyncTopicNamePostfix(
    const std::string & original_topic_name, const std::string & postfix);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CONCATENATE_AND_TIME_SYNC_NODE_HPP_
