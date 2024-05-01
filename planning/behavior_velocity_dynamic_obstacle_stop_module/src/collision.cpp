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

#include "collision.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry.hpp>

#include <limits>
#include <optional>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

std::optional<geometry_msgs::msg::Point> find_closest_collision_point(
  const EgoData & ego_data, const geometry_msgs::msg::Pose & object_pose,
  const tier4_autoware_utils::Polygon2d & object_footprint)
{
  std::optional<geometry_msgs::msg::Point> closest_collision_point;
  auto closest_dist = std::numeric_limits<double>::max();
  std::vector<BoxIndexPair> rough_collisions;
  ego_data.rtree.query(
    boost::geometry::index::intersects(object_footprint), std::back_inserter(rough_collisions));
  for (const auto & rough_collision : rough_collisions) {
    const auto path_idx = rough_collision.second;
    const auto & ego_footprint = ego_data.path_footprints[path_idx];
    const auto & ego_pose = ego_data.path.points[path_idx].point.pose;
    const auto angle_diff = tier4_autoware_utils::normalizeRadian(
      tf2::getYaw(ego_pose.orientation) - tf2::getYaw(object_pose.orientation));
    if (std::abs(angle_diff) > (M_PI_2 + M_PI_4)) {  // TODO(Maxime): make this angle a parameter
      tier4_autoware_utils::MultiPoint2d collision_points;
      boost::geometry::intersection(
        object_footprint.outer(), ego_footprint.outer(), collision_points);
      for (const auto & coll_p : collision_points) {
        auto p = geometry_msgs::msg::Point().set__x(coll_p.x()).set__y(coll_p.y());
        const auto dist_to_ego =
          motion_utils::calcSignedArcLength(ego_data.path.points, ego_data.pose.position, p);
        if (dist_to_ego < closest_dist) {
          closest_dist = dist_to_ego;
          closest_collision_point = p;
        }
      }
    }
  }
  return closest_collision_point;
}

std::vector<Collision> find_collisions(
  const EgoData & ego_data,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const tier4_autoware_utils::MultiPolygon2d & object_forward_footprints)
{
  std::vector<Collision> collisions;
  for (auto object_idx = 0UL; object_idx < objects.size(); ++object_idx) {
    const auto & object_pose = objects[object_idx].kinematics.initial_pose_with_covariance.pose;
    const auto & object_footprint = object_forward_footprints[object_idx];
    const auto collision = find_closest_collision_point(ego_data, object_pose, object_footprint);
    if (collision) {
      Collision c;
      c.object_uuid = tier4_autoware_utils::toHexString(objects[object_idx].object_id);
      c.point = *collision;
      collisions.push_back(c);
    }
  }
  return collisions;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
