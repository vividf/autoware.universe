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
#ifndef AUTOWARE__LIDAR_CENTERPOINT__TTA_PROCESSOR_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__TTA_PROCESSOR_HPP_

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/tta_config.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace autoware::lidar_centerpoint
{

/**
 * @brief Single TTA augmentation result
 */
struct TTAResult
{
  // Augmented point cloud data
  std::vector<float> augmented_points;

  // Transformation matrix
  Eigen::Matrix4f transform_matrix;

  // Inverse transformation matrix
  Eigen::Matrix4f inverse_transform_matrix;

  // Detection results for this augmentation
  std::vector<Box3D> detected_boxes;

  TTAResult()
  : transform_matrix(Eigen::Matrix4f::Identity()),
    inverse_transform_matrix(Eigen::Matrix4f::Identity())
  {
  }
};

/**
 * @brief TTA Processor class for parallel processing
 */
class TTAProcessor
{
public:
  /**
   * @brief Constructor
   * @param config TTA configuration
   * @param centerpoint_config CenterPoint configuration
   */
  TTAProcessor(const TTAConfig & config, const CenterPointConfig & centerpoint_config);

  /**
   * @brief Destructor
   */
  ~TTAProcessor() = default;

  /**
   * @brief Augment point cloud with TTA rotations
   * @param input_points Input point cloud
   * @param num_points Number of points
   * @return Vector of TTA results
   */
  std::vector<TTAResult> augmentPointCloud(const float * input_points, std::size_t num_points);

  /**
   * @brief Merge multiple TTA results using NMS
   * @param tta_results TTA results
   * @return Merged detection results
   */
  std::vector<Box3D> mergeTTAResults(const std::vector<TTAResult> & tta_results);

  /**
   * @brief Generate rotation transformation matrix
   * @param angle_degrees Rotation angle in degrees
   * @return Transformation matrix
   */
  Eigen::Matrix4f generateRotationTransform(float angle_degrees);

  /**
   * @brief Apply transformation to point cloud
   * @param input_points Input points
   * @param num_points Number of points
   * @param transform Transformation matrix
   * @param output_points Output points
   */
  void applyTransform(
    const float * input_points, std::size_t num_points, const Eigen::Matrix4f & transform,
    float * output_points);

  /**
   * @brief Transform detection boxes back to original coordinate system
   * @param boxes Detection boxes
   * @param inverse_transform Inverse transformation matrix
   * @return Transformed boxes
   */
  std::vector<Box3D> transformBoxes(
    const std::vector<Box3D> & boxes, const Eigen::Matrix4f & inverse_transform);

  /**
   * @brief Non-maximum suppression merge
   * @param all_boxes All detection boxes
   * @param iou_threshold IoU threshold
   * @return Merged boxes
   */
  std::vector<Box3D> nmsMerge(const std::vector<Box3D> & all_boxes, float iou_threshold = 0.5f);

  /**
   * @brief Calculate IoU between two 3D boxes
   * @param box1 First box
   * @param box2 Second box
   * @return IoU value
   */
  float calculateIoU(const Box3D & box1, const Box3D & box2);

  /**
   * @brief Check if TTA is enabled
   * @return True if enabled
   */
  bool isEnabled() const { return config_.enabled; }

  /**
   * @brief Get number of augmentations
   * @return Number of augmentations
   */
  int getNumAugmentations() const { return config_.num_augmentations; }

private:
  TTAConfig config_;
  CenterPointConfig centerpoint_config_;

  /**
   * @brief Convert degrees to radians
   * @param degrees Angle in degrees
   * @return Angle in radians
   */
  float degreesToRadians(float degrees) const;
};

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__TTA_PROCESSOR_HPP_
