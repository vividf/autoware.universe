// Copyright 2023-2024 TIER IV, Inc.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <rclcpp/time.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{
using BoxIndexPair = std::pair<tier4_autoware_utils::Box2d, size_t>;
using Rtree = boost::geometry::index::rtree<BoxIndexPair, boost::geometry::index::rstar<16, 4>>;

/// @brief parameters for the "out of lane" module
struct PlannerParam
{
  double extra_object_width;
  double minimum_object_velocity;
  double stop_distance_buffer;
  double time_horizon;
  double hysteresis;
  double add_duration_buffer;
  double remove_duration_buffer;
  double ego_longitudinal_offset;
  double ego_lateral_offset;
  double minimum_object_distance_from_ego_path;
  bool ignore_unavoidable_collisions;
};

struct EgoData
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path{};
  size_t first_path_idx{};
  double longitudinal_offset_to_first_path_idx;  // [m]
  geometry_msgs::msg::Pose pose;
  tier4_autoware_utils::MultiPolygon2d path_footprints;
  Rtree rtree;
  std::optional<geometry_msgs::msg::Pose> earliest_stop_pose;
};

/// @brief debug data
struct DebugData
{
  tier4_autoware_utils::MultiPolygon2d obstacle_footprints{};
  size_t prev_dynamic_obstacles_nb{};
  tier4_autoware_utils::MultiPolygon2d ego_footprints{};
  size_t prev_ego_footprints_nb{};
  std::optional<geometry_msgs::msg::Pose> stop_pose{};
  size_t prev_collisions_nb{};
  double z{};
  void reset_data()
  {
    obstacle_footprints.clear();
    ego_footprints.clear();
    stop_pose.reset();
  }
};

struct Collision
{
  geometry_msgs::msg::Point point;
  std::string object_uuid;
};
}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // TYPES_HPP_
