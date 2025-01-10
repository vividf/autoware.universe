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

#include "autoware/image_projection_based_fusion/fusion_collector.hpp"
#include "autoware/image_projection_based_fusion/fusion_matching_strategy.hpp"
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{

template <class Msg3D, class Msg2D, class ExportObj>
NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::NaiveMatchingStrategy(rclcpp::Node & node)
{
  RCLCPP_INFO(node.get_logger(), "Utilize naive matching strategy for fusion nodes.");
}


template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_rois_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const RoisMatchingParams & params) const
{
  // std::optional<double> smallest_time_difference = std::nullopt;
  // std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> closest_collector = nullptr;

  // for (const auto & cloud_collector : fusion_collectors) {
  //   if (!cloud_collector->topic_exists(params.topic_name)) {
  //     auto info = cloud_collector->get_info();
  //     if (auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(info)) {
  //       double time_difference = std::abs(params.cloud_arrival_time - naive_info->timestamp);
  //       if (!smallest_time_difference || time_difference < smallest_time_difference) {
  //         smallest_time_difference = time_difference;
  //         closest_collector = cloud_collector;
  //       }
  //     }
  //   }
  // }

  // return closest_collector;

  ////
  // for (const auto & fusion_collector : fusion_collectors_) {
  //   auto reference_timestamp_min = fusion_collector->timestamp - fusion_collector->noise_window;
  //   auto reference_timestamp_max = fusion_collector->timestamp  + fusion_collector->noise_window;
  //   double time = rois_timesatmp - id_to_offset_map_.at(roi_i);
  //   if (
  //     time < reference_timestamp_max + id_to_noise_window_map_.at(roi_i) &&
  //     time > reference_timestamp_min - id_to_noise_window_map_.at(roi_i)) {
  //     return fusion_collector;
  //   }
  // }
  // return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_det3d_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const Det3dMatchingParams & params) const
{
  // std::optional<double> smallest_time_difference = std::nullopt;
  // std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> closest_collector = nullptr;

  // for (const auto & cloud_collector : fusion_collectors) {
  //   if (!cloud_collector->topic_exists(params.topic_name)) {
  //     auto info = cloud_collector->get_info();
  //     if (auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(info)) {
  //       double time_difference = std::abs(params.cloud_arrival_time - naive_info->timestamp);
  //       if (!smallest_time_difference || time_difference < smallest_time_difference) {
  //         smallest_time_difference = time_difference;
  //         closest_collector = cloud_collector;
  //       }
  //     }
  //   }
  // }

  // return closest_collector;

  ////
  // for (const auto & fusion_collector : fusion_collectors_) {
  //   auto reference_timestamp_min = fusion_collector->timestamp - fusion_collector->noise_window;
  //   auto reference_timestamp_max = fusion_collector->timestamp  + fusion_collector->noise_window;
  //   double time = rois_timesatmp - id_to_offset_map_.at(roi_i);
  //   if (
  //     time < reference_timestamp_max + id_to_noise_window_map_.at(roi_i) &&
  //     time > reference_timestamp_min - id_to_noise_window_map_.at(roi_i)) {
  //     return fusion_collector;
  //   }
  // }
  // return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
void NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>::set_collector_info(
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector, const std::shared_ptr<MatchingParamsBase> & matching_params)
{
  auto info = std::make_shared<NaiveCollectorInfo>(matching_params.cloud_arrival_time);
  collector->set_info(info);
}


template <class Msg3D, class Msg2D, class ExportObj>
AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::AdvancedMatchingStrategy(
  rclcpp::Node & node, std::vector<std::string> input_topics)
{
  auto lidar_timestamp_offsets =
    node.declare_parameter<std::vector<double>>("matching_strategy.lidar_timestamp_offsets");
  auto lidar_timestamp_noise_window =
    node.declare_parameter<std::vector<double>>("matching_strategy.lidar_timestamp_noise_window");

  if (lidar_timestamp_offsets.size() != input_topics.size()) {
    throw std::runtime_error(
      "The number of topics does not match the number of timestamp offsets.");
  }
  if (lidar_timestamp_noise_window.size() != input_topics.size()) {
    throw std::runtime_error(
      "The number of topics does not match the number of timestamp noise window.");
  }

  for (size_t i = 0; i < input_topics.size(); i++) {
    topic_to_offset_map_[input_topics[i]] = lidar_timestamp_offsets[i];
    topic_to_noise_window_map_[input_topics[i]] = lidar_timestamp_noise_window[i];
  }

  input_topics_ = input_topics;

  RCLCPP_INFO(node.get_logger(), "Utilize advanced matching strategy for fusion nodes.");
}


template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_rois_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const RoisMatchingParams & params) const
{
  ////
  // for (const auto & fusion_collector : fusion_collectors_) {
  //   auto reference_timestamp_min = fusion_collector->timestamp - fusion_collector->noise_window;
  //   auto reference_timestamp_max = fusion_collector->timestamp  + fusion_collector->noise_window;
  //   double time = rois_timesatmp - id_to_offset_map_.at(roi_i);
  //   if (
  //     time < reference_timestamp_max + id_to_noise_window_map_.at(roi_i) &&
  //     time > reference_timestamp_min - id_to_noise_window_map_.at(roi_i)) {
  //     return fusion_collector;
  //   }
  // }
  // return std::nullopt;
}




template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::match_det3d_to_collector(
  const std::list<std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>>> & fusion_collectors,
  const Det3dMatchingParams & params) const
{
  for (const auto & cloud_collector : fusion_collectors) {
    auto info = cloud_collector->get_info();
    if (auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(info)) {
      auto reference_timestamp_min = advanced_info->timestamp - advanced_info->noise_window;
      auto reference_timestamp_max = advanced_info->timestamp + advanced_info->noise_window;
      double time = params.cloud_timestamp - topic_to_offset_map_.at(params.topic_name);
      if (
        time < reference_timestamp_max + topic_to_noise_window_map_.at(params.topic_name) &&
        time > reference_timestamp_min - topic_to_noise_window_map_.at(params.topic_name)) {
        return cloud_collector;
      }
    }
  }
  return std::nullopt;


  ///////
  if(concatenated_status) {
    auto status_map = concatenated_status.value(); // Retrieve the inner map
    if(status_map["cloud concatenation success"] == "False" || 
    (det3d_timestamp > std::stod(status_map["reference_timestamp_min"]) && 
    det3d_timestamp < std::stod(status_map["reference_timestamp_max"]))) {
      // The defined earliest pointcloud is missed in the concatenation of pointcloud
      
      std::cout << "ho" << std::endl;
    }
  }

  for (const auto & fusion_collector : fusion_collectors_) {
    auto reference_timestamp_min = fusion_collector->timestamp - fusion_collector->noise_window;
    auto reference_timestamp_max = fusion_collector->timestamp + fusion_collector->noise_window;
    if (
      det3d_timestamp < reference_timestamp_max + cloud_noise_window_ &&
      det3d_timestamp > reference_timestamp_min - cloud_noise_window_) {
      return fusion_collector;
    }
  }
  return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
void AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>::set_collector_info(
  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> & collector, const std::shared_ptr<MatchingParamsBase> & matching_params)
{
  if(auto det3d_matching_params = std::dynamic_pointer_cast<Det3dMatchingParams>(matching_params)) {
    auto info = std::make_shared<AdvancedCollectorInfo>(
      det3d_matching_params->det3d_timestamp, det3d_noise_window_);
    collector->set_info(info); 
  }
  else if(auto rois_matching_params = std::dynamic_pointer_cast<RoisMatchingParams>(matching_params)) {
    auto info = std::make_shared<AdvancedCollectorInfo>(
      rois_matching_params->rois_timestamp - topic_to_offset_map_[rois_matching_params->rois_id],
      topic_to_noise_window_map_[rois_matching_params->rois_id]);
    collector->set_info(info); 
  }
}

}  // namespace autoware::image_projection_based_fusion
