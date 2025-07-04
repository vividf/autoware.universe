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
#ifndef AUTOWARE__LIDAR_CENTERPOINT__TTA_CONFIG_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__TTA_CONFIG_HPP_

#include <vector>

namespace autoware::lidar_centerpoint
{

/**
 * @brief TTA Configuration structure
 */
struct TTAConfig
{
  // Enable TTA
  bool enabled = false;

  // Rotation angles in degrees for TTA
  std::vector<float> rotation_angles = {0.0f, 90.0f, 180.0f};

  // Number of augmentations (should match rotation_angles size)
  int num_augmentations = 3;

  // IoU threshold for NMS merging
  float iou_threshold = 0.5f;

  TTAConfig() = default;

  TTAConfig(bool enabled, const std::vector<float> & rotation_angles, float iou_threshold = 0.5f)
  : enabled(enabled && !rotation_angles.empty()),
    rotation_angles(rotation_angles),
    num_augmentations(rotation_angles.size()),
    iou_threshold(iou_threshold)
  {
  }
};

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__TTA_CONFIG_HPP_
