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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"

#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tuple>

namespace autoware::pointcloud_preprocessor
{

CloudCollector::CloudCollector(
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node,
  std::list<std::shared_ptr<CloudCollector>> & collectors,
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler, int num_of_clouds, double timeout_sec)
: concatenate_node_(concatenate_node),
  collectors_(collectors),
  combine_cloud_handler_(combine_cloud_handler),
  num_of_clouds_(num_of_clouds),
  timeout_sec_(timeout_sec)
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec_));

  timer_ = rclcpp::create_timer(
    concatenate_node_, concatenate_node_->get_clock(), period_ns,
    std::bind(&CloudCollector::concatenateCallback, this));
}

void CloudCollector::setReferenceTimeStamp(double timestamp, double noise_window)
{
  reference_timestamp_max_ = timestamp + noise_window;
  reference_timestamp_min_ = timestamp - noise_window;
}

std::tuple<double, double> CloudCollector::getReferenceTimeStampBoundary()
{
  return std::make_tuple(reference_timestamp_min_, reference_timestamp_max_);
}

void CloudCollector::processCloud(
  std::string topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  // Check if the map already contains an entry for the same topic, shouldn't happened if the
  // parameter set correctly.
  if (topic_to_cloud_map_.find(topic_name) != topic_to_cloud_map_.end()) {
    RCLCPP_WARN(
      concatenate_node_->get_logger(),
      "Topic '%s' already exists in the collector. Check the timestamp of the pointcloud.",
      topic_name.c_str());
  }
  topic_to_cloud_map_[topic_name] = cloud;
  if (topic_to_cloud_map_.size() == num_of_clouds_) {
    concatenateCallback();
  }
}

void CloudCollector::concatenateCallback()
{
  // lock for protecting collector list and concatenated pointcloud
  std::lock_guard<std::mutex> lock(mutex_);
  auto [concatenate_cloud_ptr, topic_to_transformed_cloud_map, topic_to_original_stamp_map] =
    concatenateClouds(topic_to_cloud_map_);
  publishClouds(concatenate_cloud_ptr, topic_to_transformed_cloud_map, topic_to_original_stamp_map);
  deleteCollector();
}

std::tuple<
  sensor_msgs::msg::PointCloud2::SharedPtr,
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>,
  std::unordered_map<std::string, double>>
CloudCollector::concatenateClouds(
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map)
{
  return combine_cloud_handler_->combinePointClouds(topic_to_cloud_map);
}

void CloudCollector::publishClouds(
  sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr,
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
    topic_to_transformed_cloud_map,
  std::unordered_map<std::string, double> topic_to_original_stamp_map)
{
  concatenate_node_->publishClouds(
    concatenate_cloud_ptr, topic_to_transformed_cloud_map, topic_to_original_stamp_map,
    reference_timestamp_min_, reference_timestamp_max_);
}

void CloudCollector::deleteCollector()
{
  auto it = std::find_if(
    collectors_.begin(), collectors_.end(),
    [this](const std::shared_ptr<CloudCollector> & collector) { return collector.get() == this; });
  if (it != collectors_.end()) {
    collectors_.erase(it);
  }
}

std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
CloudCollector::get_topic_to_cloud_map()
{
  return topic_to_cloud_map_;
}

}  // namespace autoware::pointcloud_preprocessor
