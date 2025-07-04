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

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_constants.h>

namespace autoware::lidar_centerpoint
{

/**
 * @brief CUDA kernel for parallel TTA point cloud rotation
 * @param input_points Input point cloud data
 * @param num_points Number of points
 * @param point_feature_size Point feature size (usually 4: x, y, z, intensity)
 * @param rotation_angles Rotation angles in radians for each augmentation
 * @param num_augmentations Number of augmentations
 * @param output_points Output augmented point clouds
 */
__global__ void rotatePointsParallelKernel(
  const float * input_points, const unsigned int num_points, const unsigned int point_feature_size,
  const float * rotation_angles, const unsigned int num_augmentations, float * output_points)
{
  const unsigned int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int aug_idx = blockIdx.y * blockDim.y + threadIdx.y;

  if (point_idx >= num_points || aug_idx >= num_augmentations) {
    return;
  }

  const float angle = rotation_angles[aug_idx];
  const float cos_angle = cosf(angle);
  const float sin_angle = sinf(angle);

  const unsigned int input_offset = point_idx * point_feature_size;
  const unsigned int output_offset = (aug_idx * num_points + point_idx) * point_feature_size;

  // Read input point
  const float x = input_points[input_offset + 0];
  const float y = input_points[input_offset + 1];
  const float z = input_points[input_offset + 2];
  const float intensity = input_points[input_offset + 3];

  // Apply rotation around Z-axis
  const float rotated_x = x * cos_angle - y * sin_angle;
  const float rotated_y = x * sin_angle + y * cos_angle;

  // Write output point
  output_points[output_offset + 0] = rotated_x;
  output_points[output_offset + 1] = rotated_y;
  output_points[output_offset + 2] = z;
  output_points[output_offset + 3] = intensity;
}

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
  cudaStream_t stream)
{
  const unsigned int block_size_x = 256;
  const unsigned int block_size_y = 1;

  const unsigned int grid_size_x = (num_points + block_size_x - 1) / block_size_x;
  const unsigned int grid_size_y = num_augmentations;

  dim3 block_dim(block_size_x, block_size_y);
  dim3 grid_dim(grid_size_x, grid_size_y);

  rotatePointsParallelKernel<<<grid_dim, block_dim, 0, stream>>>(
    input_points, num_points, point_feature_size, rotation_angles, num_augmentations,
    output_points);
}

/**
 * @brief CUDA kernel for parallel voxel generation for TTA
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
 */
__global__ void generateVoxelsTTAParallelKernel(
  const float * points, const unsigned int num_points, const unsigned int point_feature_size,
  const unsigned int num_augmentations, const float range_min_x, const float range_max_x,
  const float range_min_y, const float range_max_y, const float range_min_z,
  const float range_max_z, const float voxel_size_x, const float voxel_size_y,
  const float voxel_size_z, const unsigned int grid_size_y, const unsigned int grid_size_x,
  const unsigned int max_voxel_size, const unsigned int max_point_in_voxel_size,
  unsigned int * mask, float * voxels_buffer, unsigned int * num_voxels, float * voxels,
  float * num_points_per_voxel, int * coordinates)
{
  const unsigned int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  const unsigned int aug_idx = blockIdx.y * blockDim.y + threadIdx.y;

  if (point_idx >= num_points || aug_idx >= num_augmentations) {
    return;
  }

  const unsigned int aug_offset = aug_idx * num_points * point_feature_size;
  const unsigned int point_offset = aug_offset + point_idx * point_feature_size;

  const float x = points[point_offset + 0];
  const float y = points[point_offset + 1];
  const float z = points[point_offset + 2];

  // Check if point is within range
  if (
    x < range_min_x || x > range_max_x || y < range_min_y || y > range_max_y || z < range_min_z ||
    z > range_max_z) {
    return;
  }

  // Calculate voxel coordinates
  const int voxel_x = static_cast<int>((x - range_min_x) / voxel_size_x);
  const int voxel_y = static_cast<int>((y - range_min_y) / voxel_size_y);
  const int voxel_z = static_cast<int>((z - range_min_z) / voxel_size_z);

  if (
    voxel_x < 0 || voxel_x >= static_cast<int>(grid_size_x) || voxel_y < 0 ||
    voxel_y >= static_cast<int>(grid_size_y) || voxel_z < 0 ||
    voxel_z >= static_cast<int>(max_point_in_voxel_size)) {
    return;
  }

  const unsigned int voxel_idx = voxel_y * grid_size_x + voxel_x;
  const unsigned int mask_offset = aug_idx * grid_size_x * grid_size_y;
  const unsigned int voxel_buffer_offset =
    aug_idx * grid_size_x * grid_size_y * max_point_in_voxel_size * point_feature_size;

  // Set mask
  mask[mask_offset + voxel_idx] = 1;

  // Add point to voxel buffer
  const unsigned int buffer_idx = voxel_buffer_offset +
                                  voxel_idx * max_point_in_voxel_size * point_feature_size +
                                  voxel_z * point_feature_size;

  voxels_buffer[buffer_idx + 0] = x;
  voxels_buffer[buffer_idx + 1] = y;
  voxels_buffer[buffer_idx + 2] = z;
  voxels_buffer[buffer_idx + 3] = points[point_offset + 3];  // intensity
}

/**
 * @brief Launch function for parallel voxel generation for TTA
 */
void generateVoxelsTTAParallel_launch(
  const float * points, const unsigned int num_points, const unsigned int point_feature_size,
  const unsigned int num_augmentations, const float range_min_x, const float range_max_x,
  const float range_min_y, const float range_max_y, const float range_min_z,
  const float range_max_z, const float voxel_size_x, const float voxel_size_y,
  const float voxel_size_z, const unsigned int grid_size_y, const unsigned int grid_size_x,
  const unsigned int max_voxel_size, const unsigned int max_point_in_voxel_size,
  unsigned int * mask, float * voxels_buffer, unsigned int * num_voxels, float * voxels,
  float * num_points_per_voxel, int * coordinates, cudaStream_t stream)
{
  const unsigned int block_size_x = 256;
  const unsigned int block_size_y = 1;

  const unsigned int cuda_grid_size_x = (num_points + block_size_x - 1) / block_size_x;
  const unsigned int cuda_grid_size_y = num_augmentations;

  dim3 block_dim(block_size_x, block_size_y);
  dim3 grid_dim(cuda_grid_size_x, cuda_grid_size_y);

  generateVoxelsTTAParallelKernel<<<grid_dim, block_dim, 0, stream>>>(
    points, num_points, point_feature_size, num_augmentations, range_min_x, range_max_x,
    range_min_y, range_max_y, range_min_z, range_max_z, voxel_size_x, voxel_size_y, voxel_size_z,
    grid_size_y, grid_size_x, max_voxel_size, max_point_in_voxel_size, mask, voxels_buffer,
    num_voxels, voxels, num_points_per_voxel, coordinates);
}

}  // namespace autoware::lidar_centerpoint
