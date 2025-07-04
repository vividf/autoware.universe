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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__TTA_KERNEL_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__TTA_KERNEL_HPP_

#include <cuda_runtime.h>

namespace autoware::lidar_centerpoint
{

/**
 * @brief Launch function for parallel TTA point cloud rotation
 * @param input_points Input point cloud data
 * @param num_points Number of points
 * @param point_feature_size Point feature size
 * @param rotation_angles Rotation angles in radians
 * @param num_augmentations Number of augmentations
 * @param output_points Output augmented point clouds
 * @param stream CUDA stream
 */
void rotatePointsParallel_launch(
  const float * input_points, const unsigned int num_points, const unsigned int point_feature_size,
  const float * rotation_angles, const unsigned int num_augmentations, float * output_points,
  cudaStream_t stream);

/**
 * @brief Launch function for parallel voxel generation for TTA
 * @param points Input point cloud data for all augmentations
 * @param num_points Number of points per augmentation
 * @param point_feature_size Point feature size
 * @param num_augmentations Number of augmentations
 * @param range_min_x Minimum X range
 * @param range_max_x Maximum X range
 * @param range_min_y Minimum Y range
 * @param range_max_y Maximum Y range
 * @param range_min_z Minimum Z range
 * @param range_max_z Maximum Z range
 * @param voxel_size_x Voxel size in X
 * @param voxel_size_y Voxel size in Y
 * @param voxel_size_z Voxel size in Z
 * @param grid_size_y Grid size in Y
 * @param grid_size_x Grid size in X
 * @param max_voxel_size Maximum number of voxels
 * @param max_point_in_voxel_size Maximum points per voxel
 * @param mask Output mask for each augmentation
 * @param voxels_buffer Output voxel buffer for each augmentation
 * @param num_voxels Output number of voxels for each augmentation
 * @param voxels Output voxels for each augmentation
 * @param num_points_per_voxel Output number of points per voxel for each augmentation
 * @param coordinates Output coordinates for each augmentation
 * @param stream CUDA stream
 */
void generateVoxelsTTAParallel_launch(
  const float * points, const unsigned int num_points, const unsigned int point_feature_size,
  const unsigned int num_augmentations, const float range_min_x, const float range_max_x,
  const float range_min_y, const float range_max_y, const float range_min_z,
  const float range_max_z, const float voxel_size_x, const float voxel_size_y,
  const float voxel_size_z, const unsigned int grid_size_y, const unsigned int grid_size_x,
  const unsigned int max_voxel_size, const unsigned int max_point_in_voxel_size,
  unsigned int * mask, float * voxels_buffer, unsigned int * num_voxels, float * voxels,
  float * num_points_per_voxel, int * coordinates, cudaStream_t stream);

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__TTA_KERNEL_HPP_
