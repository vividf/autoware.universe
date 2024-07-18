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

#include "pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// debug
#include <chrono>

namespace pointcloud_preprocessor
{

CombineCloudHandler::CombineCloudHandler(
  rclcpp::Node * node, std::vector<std::string> input_topics, std::string output_frame,
  bool is_motion_compensated, bool keep_input_frame_in_synchronized_pointcloud)
: node_(node),
  tf_buffer_(node_->get_clock()),
  tf_listener_(tf_buffer_),
  input_topics_(input_topics),
  output_frame_(output_frame),
  is_motion_compensated_(is_motion_compensated),
  keep_input_frame_in_synchronized_pointcloud_(keep_input_frame_in_synchronized_pointcloud)
{
  for (auto topic : input_topics) {
    topic_to_original_stamp_map_.insert(std::make_pair(topic, -1));
    topic_to_transformed_cloud_map_.insert(std::make_pair(topic, nullptr));
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

void CombineCloudHandler::convertToXYZIRCCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
  sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr)
{
  output_ptr->header = input_ptr->header;

  PointCloud2Modifier<PointXYZIRC, autoware_point_types::PointXYZIRCGenerator> output_modifier{
    *output_ptr, input_ptr->header.frame_id};
  output_modifier.reserve(input_ptr->width);

  bool has_valid_intensity =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_return_type =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "return_type" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_channel =
    std::any_of(input_ptr->fields.begin(), input_ptr->fields.end(), [](auto & field) {
      return field.name == "channel" && field.datatype == sensor_msgs::msg::PointField::UINT16;
    });

  sensor_msgs::PointCloud2Iterator<float> it_x(*input_ptr, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*input_ptr, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*input_ptr, "z");

  if (has_valid_intensity && has_valid_return_type && has_valid_channel) {
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_i(*input_ptr, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> it_r(*input_ptr, "return_type");
    sensor_msgs::PointCloud2Iterator<std::uint16_t> it_c(*input_ptr, "channel");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      point.return_type = *it_r;
      point.channel = *it_c;
      output_modifier.push_back(std::move(point));
    }
  } else {
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
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
  std::for_each(
    std::begin(topic_to_original_stamp_map_), std::end(topic_to_original_stamp_map_),
    [](auto & pair) { pair.second = -1; });
}

void CombineCloudHandler::combinePointClouds(
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> & topic_to_cloud_map_)
{
  std::vector<rclcpp::Time> pc_stamps;
  for (const auto & pair : topic_to_cloud_map_) {
    pc_stamps.push_back(rclcpp::Time(pair.second->header.stamp));
  }

  std::sort(pc_stamps.begin(), pc_stamps.end(), std::greater<rclcpp::Time>());
  const auto oldest_stamp = pc_stamps.back();

  std::unordered_map<rclcpp::Time, Eigen::Matrix4f, RclcppTimeHash_> transform_memo;

  for (const auto & pair : topic_to_cloud_map_) {
    std::string topic = pair.first;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud = pair.second;

    auto transformed_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
    if (output_frame_ != cloud->header.frame_id) {
      if (!pcl_ros::transformPointCloud(
            output_frame_, *cloud, *transformed_cloud_ptr, tf_buffer_)) {
        RCLCPP_ERROR(
          node_->get_logger(), "Transform pointcloud from %s to %s failed.",
          cloud->header.frame_id.c_str(), output_frame_.c_str());
        return;
      }
    } else {
      transformed_cloud_ptr = cloud;
    }

    topic_to_original_stamp_map_[topic] = rclcpp::Time(cloud->header.stamp).seconds();

    auto transformed_delay_compensated_cloud_ptr =
      std::make_shared<sensor_msgs::msg::PointCloud2>();

    if (is_motion_compensated_) {
      Eigen::Matrix4f adjust_to_old_data_transform = Eigen::Matrix4f::Identity();
      rclcpp::Time current_cloud_stamp = rclcpp::Time(cloud->header.stamp);
      for (const auto & stamp : pc_stamps) {
        if (stamp >= current_cloud_stamp) continue;

        Eigen::Matrix4f new_to_old_transform;
        if (transform_memo.find(stamp) != transform_memo.end()) {
          new_to_old_transform = transform_memo[stamp];
        } else {
          new_to_old_transform =
            computeTransformToAdjustForOldTimestamp(stamp, current_cloud_stamp);
          transform_memo[stamp] = new_to_old_transform;
        }
        adjust_to_old_data_transform = new_to_old_transform * adjust_to_old_data_transform;
        current_cloud_stamp = stamp;
      }
      pcl_ros::transformPointCloud(
        adjust_to_old_data_transform, *transformed_cloud_ptr,
        *transformed_delay_compensated_cloud_ptr);

    } else {
      transformed_delay_compensated_cloud_ptr = transformed_cloud_ptr;
    }

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
      topic_to_transformed_cloud_map_[topic] = transformed_cloud_ptr_in_sensor_frame;
    } else {
      transformed_delay_compensated_cloud_ptr->header.stamp = oldest_stamp;
      transformed_delay_compensated_cloud_ptr->header.frame_id = output_frame_;
      topic_to_transformed_cloud_map_[topic] = transformed_delay_compensated_cloud_ptr;
    }
  }
  concatenate_cloud_ptr_->header.stamp = oldest_stamp;
}

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

  auto prev_time = old_stamp;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  tf2::Quaternion baselink_quat{};
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
    x += dis * std::cos(yaw);
    y += dis * std::sin(yaw);
    prev_time = (*twist_ptr_it)->header.stamp;
  }

  Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

  float cos_yaw = std::cos(yaw);
  float sin_yaw = std::sin(yaw);

  transformation_matrix(0, 3) = x;
  transformation_matrix(1, 3) = y;
  transformation_matrix(0, 0) = cos_yaw;
  transformation_matrix(0, 1) = -sin_yaw;
  transformation_matrix(1, 0) = sin_yaw;
  transformation_matrix(1, 1) = cos_yaw;

  return transformation_matrix;
}

std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
CombineCloudHandler::getTopicToTransformedCloudMap()
{
  return topic_to_transformed_cloud_map_;
}

std::unordered_map<std::string, double> CombineCloudHandler::getTopicToOriginalStampMap()
{
  return topic_to_original_stamp_map_;
}

sensor_msgs::msg::PointCloud2::SharedPtr CombineCloudHandler::getConcatenatePointcloud()
{
  return concatenate_cloud_ptr_;
}

// for diagnostic output
void CombineCloudHandler::setReferenceTimeStampBoundary(
  double reference_timestamp_min, double reference_timestamp_max)
{
  reference_timestamp_min_ = reference_timestamp_min;
  reference_timestamp_max_ = reference_timestamp_max;
}

std::tuple<double, double> CombineCloudHandler::getReferenceTimeStampBoundary()
{
  return std::make_tuple(reference_timestamp_min_, reference_timestamp_max_);
}

}  // namespace pointcloud_preprocessor
