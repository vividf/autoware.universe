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

#include "pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud_preprocessor
{

CombineCloudHandler::CombineCloudHandler(
  rclcpp::Node * node, std::vector<std::string> input_topics,
  bool keep_input_frame_in_synchronized_pointcloud, std::string output_frame)
: node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_),
  input_topics_(input_topics),
  keep_input_frame_in_synchronized_pointcloud_(keep_input_frame_in_synchronized_pointcloud),
  output_frame_(output_frame)
{
  for (size_t topic_id = 0; topic_id < input_topics_.size(); ++topic_id) {
    topic_to_transformed_cloud_map_.insert(std::make_pair(input_topics_[topic_id], nullptr));
  }
}

void CombineCloudHandler::processTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & input)
{
  // if rosbag restart, clear buffer
  if (!twist_ptr_queue_.empty()) {
    if (rclcpp::Time(twist_ptr_queue_.front()->header.stamp) > rclcpp::Time(input->header.stamp)) {
      std::cout << "Clearing twist_ptr_queue_ due to timestamp mismatch" << std::endl;
      twist_ptr_queue_.clear();
    }
  }

  // pop old data
  while (!twist_ptr_queue_.empty()) {
    if (
      rclcpp::Time(twist_ptr_queue_.front()->header.stamp) + rclcpp::Duration::from_seconds(1.0) >
      rclcpp::Time(input->header.stamp)) {
      break;
    }
    twist_ptr_queue_.pop_front();
  }

  auto twist_ptr = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_ptr->header = input->header;
  twist_ptr->twist = input->twist.twist;
  twist_ptr_queue_.push_back(twist_ptr);

}

void CombineCloudHandler::processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & input)
{
  // if rosbag restart, clear buffer
  if (!twist_ptr_queue_.empty()) {
    if (rclcpp::Time(twist_ptr_queue_.front()->header.stamp) > rclcpp::Time(input->header.stamp)) {
      twist_ptr_queue_.clear();
    }
  }

  // pop old data
  while (!twist_ptr_queue_.empty()) {
    if (
      rclcpp::Time(twist_ptr_queue_.front()->header.stamp) + rclcpp::Duration::from_seconds(1.0) >
      rclcpp::Time(input->header.stamp)) {
      break;
    }
    twist_ptr_queue_.pop_front();
  }

  auto twist_ptr = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_ptr->header = input->header;
  twist_ptr->twist = input->twist.twist;
  twist_ptr_queue_.push_back(twist_ptr);
}

void CombineCloudHandler::convertToXYZICloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr)
{
  output_ptr->header = input_ptr->header;

  PointCloud2Modifier<PointXYZI> output_modifier{*output_ptr, input_ptr->header.frame_id};
  output_modifier.reserve(input_ptr->width);

  bool has_intensity = std::any_of(
    input_ptr->fields.begin(), input_ptr->fields.end(),
    [](auto & field) { return field.name == "intensity"; });

  sensor_msgs::PointCloud2Iterator<float> it_x(*input_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*input_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*input_ptr, "z");

  if (has_intensity) {
    sensor_msgs::PointCloud2Iterator<float> it_i(*input_ptr, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      PointXYZI point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      output_modifier.push_back(std::move(point));
    }
  } else {
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      PointXYZI point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = 0.0f;
      output_modifier.push_back(std::move(point));
    }
  }
}

void CombineCloudHandler::resetCloud()
{
  // reset the pointcloud and map
  concatenate_cloud_ptr_ = nullptr;
  std::for_each(
    std::begin(topic_to_transformed_cloud_map_), std::end(topic_to_transformed_cloud_map_),
    [](auto & pair) { pair.second = nullptr; });
}

void CombineCloudHandler::combinePointClouds(
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
    topic_to_cloud_map_)
{
  std::vector<rclcpp::Time> pc_stamps;
  for (const auto & pair : topic_to_cloud_map_) {
    pc_stamps.push_back(rclcpp::Time(pair.second->header.stamp));
  }

  std::sort(pc_stamps.begin(), pc_stamps.end(), std::greater<rclcpp::Time>());
  const auto oldest_stamp = pc_stamps.back();

  sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr(
    new sensor_msgs::msg::PointCloud2());
  for (const auto & pair : topic_to_cloud_map_) {
    std::string topic_name = pair.first;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud = pair.second;
    std::cout << "in combination topic: " << topic_name << std::endl;

    transformPointCloud(cloud, transformed_cloud_ptr);
    // calculate transforms to oldest stamp

    // TODO(vivid): speed optimization: there is no reason we need to calculate transforms one by
    // one.
    Eigen::Matrix4f adjust_to_old_data_transform = Eigen::Matrix4f::Identity();
    rclcpp::Time transformed_stamp = rclcpp::Time(cloud->header.stamp);
    for (const auto & stamp : pc_stamps) {
      const auto new_to_old_transform =
        computeTransformToAdjustForOldTimestamp(stamp, transformed_stamp);
      adjust_to_old_data_transform = new_to_old_transform * adjust_to_old_data_transform;
      transformed_stamp = std::min(transformed_stamp, stamp);
    }
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_delay_compensated_cloud_ptr(
      new sensor_msgs::msg::PointCloud2());
    pcl_ros::transformPointCloud(
      adjust_to_old_data_transform, *transformed_cloud_ptr,
      *transformed_delay_compensated_cloud_ptr);

    // concatenate
    if (concatenate_cloud_ptr_ == nullptr) {
      concatenate_cloud_ptr_ =
        std::make_shared<sensor_msgs::msg::PointCloud2>(*transformed_delay_compensated_cloud_ptr);
    } else {
      pcl::concatenatePointCloud(
        *concatenate_cloud_ptr_, *transformed_delay_compensated_cloud_ptr, *concatenate_cloud_ptr_);
    }
    // convert to original sensor frame if necessary

    bool need_transform_to_sensor_frame = (cloud->header.frame_id != output_frame_);
    if (keep_input_frame_in_synchronized_pointcloud_ && need_transform_to_sensor_frame) {
      sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud_ptr_in_sensor_frame(
        new sensor_msgs::msg::PointCloud2());
      pcl_ros::transformPointCloud(
        (std::string)cloud->header.frame_id, *transformed_delay_compensated_cloud_ptr,
        *transformed_cloud_ptr_in_sensor_frame, tf_buffer_);
      transformed_cloud_ptr_in_sensor_frame->header.stamp = oldest_stamp;
      transformed_cloud_ptr_in_sensor_frame->header.frame_id = cloud->header.frame_id;
      topic_to_transformed_cloud_map_[topic_name] = transformed_cloud_ptr_in_sensor_frame;
    } else {
      transformed_delay_compensated_cloud_ptr->header.stamp = oldest_stamp;
      transformed_delay_compensated_cloud_ptr->header.frame_id = output_frame_;
      topic_to_transformed_cloud_map_[topic_name] = transformed_delay_compensated_cloud_ptr;
    }
  }
  concatenate_cloud_ptr_->header.stamp = oldest_stamp;
}

void CombineCloudHandler::transformPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_cloud)
{
  // Transform the point clouds into the specified output frame
  if (output_frame_ != input_cloud->header.frame_id) {
    if (!pcl_ros::transformPointCloud(output_frame_, *input_cloud, *output_cloud, tf_buffer_)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[transformPointCloud] Error converting first input dataset from %s to %s.",
        input_cloud->header.frame_id.c_str(), output_frame_.c_str());
      return;
    }
  } else {
    output_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*input_cloud);
  }
}

/**
 * @brief compute transform to adjust for old timestamp
 *
 * @param old_stamp
 * @param new_stamp
 * @return Eigen::Matrix4f: transformation matrix from new_stamp to old_stamp
 */
Eigen::Matrix4f CombineCloudHandler::computeTransformToAdjustForOldTimestamp(
  const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp)
{
  // return identity if no twist is available
  if (twist_ptr_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), std::chrono::milliseconds(10000).count(),
      "No twist is available. Please confirm twist topic and timestamp");
    return Eigen::Matrix4f::Identity();
  }

  // return identity if old_stamp is newer than new_stamp
  if (old_stamp > new_stamp) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), std::chrono::milliseconds(10000).count(),
      "old_stamp is newer than new_stamp,");
    return Eigen::Matrix4f::Identity();
  }

  auto old_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), old_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, const rclcpp::Time & t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  old_twist_ptr_it =
    old_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : old_twist_ptr_it;

  auto new_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), new_stamp,
    [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & x_ptr, const rclcpp::Time & t) {
      return rclcpp::Time(x_ptr->header.stamp) < t;
    });
  new_twist_ptr_it =
    new_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : new_twist_ptr_it;

  // TODO(vivid): this is same as distortion correction function. unify them
  auto prev_time = old_stamp;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (auto twist_ptr_it = old_twist_ptr_it; twist_ptr_it != new_twist_ptr_it + 1; ++twist_ptr_it) {
    const double dt =
      (twist_ptr_it != new_twist_ptr_it)
        ? (rclcpp::Time((*twist_ptr_it)->header.stamp) - rclcpp::Time(prev_time)).seconds()
        : (rclcpp::Time(new_stamp) - rclcpp::Time(prev_time)).seconds();

    if (std::fabs(dt) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        node_->get_logger(), *node_->get_clock(), std::chrono::milliseconds(10000).count(),
        "Time difference is too large. Cloud not interpolate. Please confirm twist topic and "
        "timestamp");
      break;
    }

    const double dis = (*twist_ptr_it)->twist.linear.x * dt;
    yaw += (*twist_ptr_it)->twist.angular.z * dt;
    // TODO(vivid): change to tier4 sin cos?
    x += dis * std::cos(yaw);
    y += dis * std::sin(yaw);
    prev_time = (*twist_ptr_it)->header.stamp;
  }
  Eigen::AngleAxisf rotation_x(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(x, y, 0);
  Eigen::Matrix4f rotation_matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();
  return rotation_matrix;
}

std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
CombineCloudHandler::getTopicToTransformedCloudMap()
{
  return topic_to_transformed_cloud_map_;
}

sensor_msgs::msg::PointCloud2::SharedPtr CombineCloudHandler::getConcatenatePointcloud()
{
  return concatenate_cloud_ptr_;
}

}  // namespace pointcloud_preprocessor
