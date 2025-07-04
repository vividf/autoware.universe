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
  // Debug logging
  std::cout << "TTAProcessor constructor called" << std::endl;
  std::cout << "TTA enabled: " << (config_.enabled ? "true" : "false") << std::endl;
  std::cout << "Number of augmentations: " << config_.num_augmentations << std::endl;
  std::cout << "Rotation angles: ";
  for (const auto & angle : config_.rotation_angles) {
    std::cout << angle << " ";
  }
  std::cout << std::endl;
}

std::vector<TTAResult> TTAProcessor::augmentPointCloud(
  const float * input_points, std::size_t num_points)
{
  std::cout << "TTAProcessor::augmentPointCloud called with " << num_points << " points"
            << std::endl;
  std::vector<TTAResult> results;

  std::cout << "Checking if TTA is enabled and rotation angles exist" << std::endl;
  if (!config_.enabled || config_.rotation_angles.empty()) {
    std::cout << "TTA disabled or no rotation angles, returning original point cloud" << std::endl;
    // Return original point cloud if TTA is disabled or no rotation angles
    TTAResult result;
    result.augmented_points.resize(num_points * centerpoint_config_.point_feature_size_);
    std::copy(
      input_points, input_points + num_points * centerpoint_config_.point_feature_size_,
      result.augmented_points.begin());
    results.push_back(result);
    return results;
  }

  std::cout << "TTA is enabled, generating augmentations for " << config_.num_augmentations
            << " angles" << std::endl;
  // Generate augmentations for each rotation angle
  for (int i = 0; i < config_.num_augmentations; ++i) {
    std::cout << "Processing augmentation " << i << " with angle " << config_.rotation_angles[i]
              << std::endl;
    TTAResult result;

    // Generate transformation matrix for this rotation
    std::cout << "Generating rotation transform" << std::endl;
    result.transform_matrix = generateRotationTransform(config_.rotation_angles[i]);
    std::cout << "Computing inverse transform" << std::endl;
    result.inverse_transform_matrix = result.transform_matrix.inverse();

    // Apply transformation to point cloud
    std::cout << "Resizing augmented points vector" << std::endl;
    result.augmented_points.resize(num_points * centerpoint_config_.point_feature_size_);
    std::cout << "Applying transform to point cloud" << std::endl;
    applyTransform(
      input_points, num_points, result.transform_matrix, result.augmented_points.data());

    std::cout << "Adding result to results vector" << std::endl;
    results.push_back(result);
  }

  std::cout << "augmentPointCloud completed, returning " << results.size() << " results"
            << std::endl;
  return results;
}

Eigen::Matrix4f TTAProcessor::generateRotationTransform(float angle_degrees)
{
  std::cout << "generateRotationTransform called with angle: " << angle_degrees << " degrees"
            << std::endl;

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Convert degrees to radians
  float angle_rad = degreesToRadians(angle_degrees);
  std::cout << "Converted to radians: " << angle_rad << std::endl;

  // Create rotation matrix around Z-axis
  Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitZ()).matrix();
  transform.block<3, 3>(0, 0) = rotation;

  std::cout << "generateRotationTransform completed" << std::endl;
  return transform;
}

void TTAProcessor::applyTransform(
  const float * input_points, std::size_t num_points, const Eigen::Matrix4f & transform,
  float * output_points)
{
  std::cout << "applyTransform called with " << num_points << " points" << std::endl;

  // Safety checks
  if (input_points == nullptr) {
    std::cout << "ERROR: input_points is null!" << std::endl;
    return;
  }

  if (output_points == nullptr) {
    std::cout << "ERROR: output_points is null!" << std::endl;
    return;
  }

  std::cout << "Input points pointer: " << input_points << std::endl;
  std::cout << "Output points pointer: " << output_points << std::endl;
  std::cout << "First few input values: " << input_points[0] << ", " << input_points[1] << ", "
            << input_points[2] << ", " << input_points[3] << std::endl;

  for (std::size_t i = 0; i < num_points; ++i) {
    if (i % 10000 == 0) {
      std::cout << "Processing point " << i << " of " << num_points << std::endl;
    }

    // Check bounds before accessing
    if (i * 4 + 3 >= num_points * 4) {
      std::cout << "ERROR: Array bounds exceeded at point " << i << std::endl;
      break;
    }

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

  std::cout << "applyTransform completed" << std::endl;
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
  std::cout << "degreesToRadians called with: " << degrees << " degrees" << std::endl;
  float result = degrees * M_PI / 180.0f;
  std::cout << "degreesToRadians result: " << result << " radians" << std::endl;
  return result;
}

}  // namespace autoware::lidar_centerpoint
