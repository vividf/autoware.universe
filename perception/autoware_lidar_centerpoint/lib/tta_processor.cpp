// Copyright 2025 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/tta_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace autoware::lidar_centerpoint
{

TTAProcessor::TTAProcessor(const TTAConfig & config, const CenterPointConfig & centerpoint_config)
: config_(config), centerpoint_config_(centerpoint_config)
{
}

std::vector<TTAResult> TTAProcessor::augmentPointCloud(
  const float * input_points, std::size_t num_points)
{
  std::vector<TTAResult> results;

  if (!config_.enabled) {
    // Return original point cloud if TTA is disabled
    TTAResult result;
    result.augmented_points.resize(num_points * centerpoint_config_.point_feature_size_);
    std::copy(
      input_points, input_points + num_points * centerpoint_config_.point_feature_size_,
      result.augmented_points.begin());
    results.push_back(result);
    return results;
  }

  // Generate augmentations for each rotation angle
  for (int i = 0; i < config_.num_augmentations; ++i) {
    TTAResult result;

    // Generate transformation matrix for this rotation
    result.transform_matrix = generateRotationTransform(config_.rotation_angles[i]);
    result.inverse_transform_matrix = result.transform_matrix.inverse();

    // Apply transformation to point cloud
    result.augmented_points.resize(num_points * centerpoint_config_.point_feature_size_);
    applyTransform(
      input_points, num_points, result.transform_matrix, result.augmented_points.data());

    results.push_back(result);
  }

  return results;
}

Eigen::Matrix4f TTAProcessor::generateRotationTransform(float angle_degrees)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Convert degrees to radians
  float angle_rad = degreesToRadians(angle_degrees);

  // Create rotation matrix around Z-axis
  Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitZ()).matrix();
  transform.block<3, 3>(0, 0) = rotation;

  return transform;
}

void TTAProcessor::applyTransform(
  const float * input_points, std::size_t num_points, const Eigen::Matrix4f & transform,
  float * output_points)
{
  for (std::size_t i = 0; i < num_points; ++i) {
    Eigen::Vector4f point;
    point << input_points[i * 4 + 0],  // x
      input_points[i * 4 + 1],         // y
      input_points[i * 4 + 2],         // z
      1.0f;                            // homogeneous coordinate

    Eigen::Vector4f transformed_point = transform * point;

    output_points[i * 4 + 0] = transformed_point(0);     // x
    output_points[i * 4 + 1] = transformed_point(1);     // y
    output_points[i * 4 + 2] = transformed_point(2);     // z
    output_points[i * 4 + 3] = input_points[i * 4 + 3];  // intensity/time
  }
}

std::vector<Box3D> TTAProcessor::transformBoxes(
  const std::vector<Box3D> & boxes, const Eigen::Matrix4f & inverse_transform)
{
  std::vector<Box3D> transformed_boxes;
  transformed_boxes.reserve(boxes.size());

  for (const auto & box : boxes) {
    Box3D transformed_box = box;

    // Transform center point
    Eigen::Vector4f center;
    center << box.x, box.y, box.z, 1.0f;
    Eigen::Vector4f transformed_center = inverse_transform * center;
    transformed_box.x = transformed_center(0);
    transformed_box.y = transformed_center(1);
    transformed_box.z = transformed_center(2);

    // Transform velocity
    Eigen::Vector4f velocity;
    velocity << box.vel_x, box.vel_y, 0.0f, 0.0f;
    Eigen::Vector4f transformed_velocity = inverse_transform * velocity;
    transformed_box.vel_x = transformed_velocity(0);
    transformed_box.vel_y = transformed_velocity(1);

    // Transform yaw angle
    Eigen::Matrix3f rotation = inverse_transform.block<3, 3>(0, 0);
    float cos_yaw = rotation(0, 0);
    float sin_yaw = rotation(1, 0);
    float additional_yaw = std::atan2(sin_yaw, cos_yaw);
    transformed_box.yaw = box.yaw + additional_yaw;

    // Normalize yaw angle to [-π, π]
    while (transformed_box.yaw > M_PI) transformed_box.yaw -= 2 * M_PI;
    while (transformed_box.yaw < -M_PI) transformed_box.yaw += 2 * M_PI;

    transformed_boxes.push_back(transformed_box);
  }

  return transformed_boxes;
}

std::vector<Box3D> TTAProcessor::mergeTTAResults(const std::vector<TTAResult> & tta_results)
{
  std::vector<Box3D> all_boxes;

  // Collect all transformed detection boxes
  for (const auto & result : tta_results) {
    auto transformed_boxes = transformBoxes(result.detected_boxes, result.inverse_transform_matrix);
    all_boxes.insert(all_boxes.end(), transformed_boxes.begin(), transformed_boxes.end());
  }

  // Use NMS to merge overlapping detections
  return nmsMerge(all_boxes, config_.iou_threshold);
}

std::vector<Box3D> TTAProcessor::nmsMerge(const std::vector<Box3D> & all_boxes, float iou_threshold)
{
  if (all_boxes.empty()) {
    return {};
  }

  // Sort by score
  std::vector<Box3D> sorted_boxes = all_boxes;
  std::sort(sorted_boxes.begin(), sorted_boxes.end(), [](const Box3D & a, const Box3D & b) {
    return a.score > b.score;
  });

  std::vector<bool> keep(sorted_boxes.size(), true);

  // Perform NMS
  for (std::size_t i = 0; i < sorted_boxes.size(); ++i) {
    if (!keep[i]) continue;

    for (std::size_t j = i + 1; j < sorted_boxes.size(); ++j) {
      if (!keep[j]) continue;

      if (calculateIoU(sorted_boxes[i], sorted_boxes[j]) > iou_threshold) {
        keep[j] = false;
      }
    }
  }

  // Collect kept boxes
  std::vector<Box3D> result;
  for (std::size_t i = 0; i < sorted_boxes.size(); ++i) {
    if (keep[i]) {
      result.push_back(sorted_boxes[i]);
    }
  }

  return result;
}

float TTAProcessor::calculateIoU(const Box3D & box1, const Box3D & box2)
{
  // Simplified 2D IoU calculation (ignoring height)
  float x1_min = box1.x - box1.length / 2;
  float x1_max = box1.x + box1.length / 2;
  float y1_min = box1.y - box1.width / 2;
  float y1_max = box1.y + box1.width / 2;

  float x2_min = box2.x - box2.length / 2;
  float x2_max = box2.x + box2.length / 2;
  float y2_min = box2.y - box2.width / 2;
  float y2_max = box2.y + box2.width / 2;

  float x_min = std::max(x1_min, x2_min);
  float x_max = std::min(x1_max, x2_max);
  float y_min = std::max(y1_min, y2_min);
  float y_max = std::min(y1_max, y2_max);

  if (x_max <= x_min || y_max <= y_min) {
    return 0.0f;  // No intersection
  }

  float intersection = (x_max - x_min) * (y_max - y_min);
  float area1 = box1.length * box1.width;
  float area2 = box2.length * box2.width;
  float union_area = area1 + area2 - intersection;

  return intersection / union_area;
}

float TTAProcessor::degreesToRadians(float degrees) const
{
  return degrees * M_PI / 180.0f;
}

}  // namespace autoware::lidar_centerpoint
