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

#ifndef POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_
#define POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include "autoware_point_types/types.hpp"

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;

class CombineCloudHandler
{
public:
  rclcpp::Node * node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::string> input_topics_;
  std::string output_frame_;
  bool is_motion_compensated_;
  bool keep_input_frame_in_synchronized_pointcloud_;

  struct RclcppTimeHash_
  {
    std::size_t operator()(const rclcpp::Time & t) const
    {
      return std::hash<int64_t>()(t.nanoseconds());
    }
  };

  sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr_;
  // TODO(vivid): also check the boolen flag
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
    topic_to_transformed_cloud_map_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

  CombineCloudHandler(
    rclcpp::Node * node, std::vector<std::string> input_topics, std::string output_frame,
    bool is_motion_compensated, bool keep_input_frame_in_synchronized_pointcloud);
  void processTwist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & input);
  void processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & input);
  void convertToXYZIRCCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
    sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr);

  void resetCloud();
  void combinePointClouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> & topic_cloud_map_);
  Eigen::Matrix4f computeTransformToAdjustForOldTimestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
  getTopicToTransformedCloudMap();
  sensor_msgs::msg::PointCloud2::SharedPtr getConcatenatePointcloud();
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__COMBINE_CLOUD_HANDLER_HPP_
