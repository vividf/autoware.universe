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

#include "pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"

#include "pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_nodelet.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
using namespace std::chrono_literals;

namespace pointcloud_preprocessor
{

CloudCollector::CloudCollector(
  rclcpp::Node * node,
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node,
  std::vector<std::shared_ptr<CloudCollector>> & collectors,
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler, int num_of_clouds)
: node_(node),
  concatenate_node_(concatenate_node),
  collectors_(collectors),
  combine_cloud_handler_(combine_cloud_handler),
  num_of_clouds_(num_of_clouds)
{
  //timer_ = concatenate_node_->create_wall_timer(
  //    std::chrono::milliseconds(100), std::bind(&CloudCollector::combineClouds2, this));

  auto timer_callback = std::bind(&CloudCollector::combineClouds2, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(0.1));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    concatenate_node->get_clock(), period_control, std::move(timer_callback),
    concatenate_node->get_node_base_interface()->get_context());
  concatenate_node->get_node_timers_interface()->add_timer(timer_, nullptr);

  // not working
  // const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
  //  std::chrono::duration<double>(0.1));
  
  // timer_ = rclcpp::create_timer(
  //  node_, node_->get_clock(), period_ns, std::bind(&CloudCollector::combineClouds2, this));


  //rclcpp::sleep_for(std::chrono::milliseconds(50));    
  std::cout << "time: " << timer_->time_until_trigger().count() << std::endl;
  //timer_ = rclcpp::create_timer(
  // concatenate_node, concatenate_node->get_clock(), std::chrono::milliseconds(100), std::bind(&CloudCollector::combineClouds2, this));
  
  //const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
  //   std::chrono::duration<double>(0.1));

  // std::chrono::nanoseconds wait_duration(static_cast<int>(1e9 * 1));
  // timer_ = rclcpp::create_timer(
  //    node, node->get_clock(), wait_duration, std::bind(&CloudCollector::combineClouds2, this));

  if (timer_ == nullptr) {
    std::cerr << "timer_ is nullptr in con" << std::endl;
  }
  else {
    std::cerr << "timer_ is not nullptr in con" << std::endl;
  }

  timestamp_ = 0.0;
}


void CloudCollector::setTimeStamp(double timestamp)
{
  timestamp_ = timestamp;
}

double CloudCollector::getTimeStamp()
{
  return timestamp_;
}

void CloudCollector::processCloud(
  std::string topic_name, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  topic_cloud_map_[topic_name] = cloud;
  std::cout << "cloud topic name: " << topic_name << std::endl;
  std::cout << "topic_cloud_map_ size: " << topic_cloud_map_.size() << std::endl;
  if (topic_cloud_map_.size() == num_of_clouds_) combineClouds1();



  if (timer_ == nullptr) {
    std::cerr << "timer_ is nullptr in process" << std::endl;
  }
  else {
    std::cerr << "timer_ is not nullptr in process" << std::endl;
  }
  std::cout << "time in process: " << timer_->time_until_trigger().count() << std::endl;
}

void CloudCollector::combineClouds1()
{
    if (timer_ == nullptr) {
    std::cerr << "timer_ is nullptr in combine" << std::endl;
  }
  else {
    std::cerr << "timer_ is not nullptr in combine" << std::endl;
  }
  std::cout << "time in 11111: " << timer_->time_until_trigger().count() << std::endl;
  std::cout << "Combining cloud from match " << std::endl;;
  combine_cloud_handler_->combinePointClouds(topic_cloud_map_);
  concatenate_node_->publishClouds();
  std::lock_guard<std::mutex> lock(mutex_);
  deleteCollector();

}


// for debugging
void CloudCollector::combineClouds2()
{

  if (timer_ == nullptr) {
    std::cerr << "timer_ is nullptr in combine" << std::endl;
  }
  else {
    std::cerr << "timer_ is not nullptr in combine" << std::endl;
  }
  std::cout << "Combining cloud from timer" << std::endl;;
  combine_cloud_handler_->combinePointClouds(topic_cloud_map_);
  concatenate_node_->publishClouds();
  std::lock_guard<std::mutex> lock(mutex_);
  deleteCollector();

}

void CloudCollector::deleteCollector()
{
  // Remove from the collectors, and will get deleted as it is an unique pointer.
  auto it = std::find_if(
    collectors_.begin(), collectors_.end(),
    [this](const std::shared_ptr<CloudCollector> & collector) { return collector.get() == this; });
  if (it != collectors_.end()) {
    collectors_.erase(it);
    std::cout << "***********erase***************" << std::endl;
  }
}

}  // namespace pointcloud_preprocessor
