// Copyright 2021 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/centerpoint_trt.hpp"

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/network/scatter_kernel.hpp"
#include "autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp"
#include "autoware/lidar_centerpoint/preprocess/tta_kernel.hpp"
#include "autoware/lidar_centerpoint/tta_processor.hpp"

#include <autoware_utils/math/constants.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::lidar_centerpoint
{
CenterPointTRT::CenterPointTRT(
  const TrtCommonConfig & encoder_param, const TrtCommonConfig & head_param,
  const DensificationParam & densification_param, const CenterPointConfig & config,
  const TTAConfig & tta_config)
: config_(config)
{
  // Create CUDA stream first, as it's needed by initPtr()
  cudaStreamCreate(&stream_);

  vg_ptr_ = std::make_unique<VoxelGenerator>(densification_param, config_);
  post_proc_ptr_ = std::make_unique<PostProcessCUDA>(config_);

  // Initialize TTA processor with configuration from parameters
  if (tta_config.enabled) {
    tta_processor_ = std::make_unique<TTAProcessor>(tta_config, config_);
  } else {
    tta_processor_ = nullptr;
  }

  initPtr();
  initTrt(encoder_param, head_param);
  initTTAMemory();
}

CenterPointTRT::~CenterPointTRT()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

void CenterPointTRT::initPtr()
{
  voxels_size_ =
    config_.max_voxel_size_ * config_.max_point_in_voxel_size_ * config_.point_feature_size_;
  coordinates_size_ = config_.max_voxel_size_ * config_.point_dim_size_;
  encoder_in_feature_size_ =
    config_.max_voxel_size_ * config_.max_point_in_voxel_size_ * config_.encoder_in_feature_size_;
  const auto pillar_features_size = config_.max_voxel_size_ * config_.encoder_out_feature_size_;
  spatial_features_size_ =
    config_.grid_size_x_ * config_.grid_size_y_ * config_.encoder_out_feature_size_;
  const auto grid_xy_size = config_.down_grid_size_x_ * config_.down_grid_size_y_;

  voxels_buffer_size_ = config_.grid_size_x_ * config_.grid_size_y_ *
                        config_.max_point_in_voxel_size_ * config_.point_feature_size_;
  mask_size_ = config_.grid_size_x_ * config_.grid_size_y_;

  // host
  points_.resize(config_.cloud_capacity_ * config_.point_feature_size_);

  // device
  voxels_d_ = cuda::make_unique<float[]>(voxels_size_);
  coordinates_d_ = cuda::make_unique<int[]>(coordinates_size_);
  num_points_per_voxel_d_ = cuda::make_unique<float[]>(config_.max_voxel_size_);
  encoder_in_features_d_ = cuda::make_unique<float[]>(encoder_in_feature_size_);
  pillar_features_d_ = cuda::make_unique<float[]>(pillar_features_size);
  spatial_features_d_ = cuda::make_unique<float[]>(spatial_features_size_);
  head_out_heatmap_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.class_size_);
  head_out_offset_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.head_out_offset_size_);
  head_out_z_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.head_out_z_size_);
  head_out_dim_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.head_out_dim_size_);
  head_out_rot_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.head_out_rot_size_);
  head_out_vel_d_ = cuda::make_unique<float[]>(grid_xy_size * config_.head_out_vel_size_);
  points_d_ = cuda::make_unique<float[]>(config_.cloud_capacity_ * config_.point_feature_size_);
  voxels_buffer_d_ = cuda::make_unique<float[]>(voxels_buffer_size_);
  mask_d_ = cuda::make_unique<unsigned int[]>(mask_size_);
  num_voxels_d_ = cuda::make_unique<unsigned int[]>(1);

  points_aux_d_ = cuda::make_unique<float[]>(config_.cloud_capacity_ * config_.point_feature_size_);
  shuffle_indices_d_ = cuda::make_unique<unsigned int[]>(config_.cloud_capacity_);

  std::vector<unsigned int> indexes(config_.cloud_capacity_);
  std::iota(indexes.begin(), indexes.end(), 0);

  std::default_random_engine e(0);
  std::shuffle(indexes.begin(), indexes.end(), e);

  std::srand(std::time(nullptr));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    shuffle_indices_d_.get(), indexes.data(), config_.cloud_capacity_ * sizeof(unsigned int),
    cudaMemcpyHostToDevice, stream_));
}

void CenterPointTRT::initTTAMemory()
{
  if (!tta_processor_ || !tta_processor_->isEnabled()) {
    return;
  }

  int num_augmentations = tta_processor_->getNumAugmentations();

  // Allocate TTA GPU memory buffers
  tta_points_d_ = cuda::make_unique<float[]>(
    config_.cloud_capacity_ * config_.point_feature_size_ * num_augmentations);
  tta_voxels_d_ = cuda::make_unique<float[]>(voxels_size_ * num_augmentations);
  tta_encoder_features_d_ =
    cuda::make_unique<float[]>(encoder_in_feature_size_ * num_augmentations);
  tta_spatial_features_d_ = cuda::make_unique<float[]>(spatial_features_size_ * num_augmentations);
  tta_head_outputs_d_ = cuda::make_unique<float[]>(
    (config_.down_grid_size_x_ * config_.down_grid_size_y_ *
     (config_.class_size_ + config_.head_out_offset_size_ + config_.head_out_z_size_ +
      config_.head_out_dim_size_ + config_.head_out_rot_size_ + config_.head_out_vel_size_)) *
    num_augmentations);
  tta_num_voxels_d_ = cuda::make_unique<unsigned int[]>(num_augmentations);
  tta_coordinates_d_ = cuda::make_unique<int[]>(coordinates_size_ * num_augmentations);
  tta_num_points_per_voxel_d_ =
    cuda::make_unique<float[]>(config_.max_voxel_size_ * num_augmentations);
}

void CenterPointTRT::initTrt(
  const TrtCommonConfig & encoder_param, const TrtCommonConfig & head_param)
{
  // encoder input profile
  auto enc_in_dims = nvinfer1::Dims{
    3,
    {static_cast<int32_t>(config_.max_voxel_size_),
     static_cast<int32_t>(config_.max_point_in_voxel_size_),
     static_cast<int32_t>(config_.encoder_in_feature_size_)}};
  std::vector<tensorrt_common::ProfileDims> encoder_profile_dims{
    tensorrt_common::ProfileDims(0, enc_in_dims, enc_in_dims, enc_in_dims)};
  auto encoder_profile_dims_ptr =
    std::make_unique<std::vector<tensorrt_common::ProfileDims>>(encoder_profile_dims);

  // head input profile
  auto head_in_dims = nvinfer1::Dims{
    4,
    {static_cast<int32_t>(config_.batch_size_),
     static_cast<int32_t>(config_.encoder_out_feature_size_),
     static_cast<int32_t>(config_.grid_size_y_), static_cast<int32_t>(config_.grid_size_x_)}};
  std::vector<tensorrt_common::ProfileDims> head_profile_dims{
    tensorrt_common::ProfileDims(0, head_in_dims, head_in_dims, head_in_dims)};
  std::unordered_map<int32_t, std::int32_t> out_channel_map = {
    {1, static_cast<int32_t>(config_.class_size_)},
    {2, static_cast<int32_t>(config_.head_out_offset_size_)},
    {3, static_cast<int32_t>(config_.head_out_z_size_)},
    {4, static_cast<int32_t>(config_.head_out_dim_size_)},
    {5, static_cast<int32_t>(config_.head_out_rot_size_)},
    {6, static_cast<int32_t>(config_.head_out_vel_size_)}};
  for (const auto & [tensor_name, channel_size] : out_channel_map) {
    auto dims = nvinfer1::Dims{
      4,
      {static_cast<int32_t>(config_.batch_size_), channel_size,
       static_cast<int32_t>(config_.down_grid_size_y_),
       static_cast<int32_t>(config_.down_grid_size_x_)}};
    head_profile_dims.emplace_back(tensor_name, dims, dims, dims);
  }
  auto head_profile_dims_ptr =
    std::make_unique<std::vector<tensorrt_common::ProfileDims>>(head_profile_dims);

  // initialize trt wrappers
  encoder_trt_ptr_ = std::make_unique<tensorrt_common::TrtCommon>(encoder_param);

  head_trt_ptr_ = std::make_unique<tensorrt_common::TrtCommon>(head_param);

  // setup trt engines
  if (
    !encoder_trt_ptr_->setup(std::move(encoder_profile_dims_ptr)) ||
    !head_trt_ptr_->setup(std::move(head_profile_dims_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine.");
  }

  // set input shapes
  if (
    !encoder_trt_ptr_->setInputShape(0, enc_in_dims) ||
    !head_trt_ptr_->setInputShape(0, head_in_dims)) {
    throw std::runtime_error("Failed to set input shape.");
  }
}

bool CenterPointTRT::detect(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & input_pointcloud_msg_ptr,
  const tf2_ros::Buffer & tf_buffer, std::vector<Box3D> & det_boxes3d,
  bool & is_num_pillars_within_range)
{
  is_num_pillars_within_range = true;

  CHECK_CUDA_ERROR(cudaMemsetAsync(
    encoder_in_features_d_.get(), 0, encoder_in_feature_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(spatial_features_d_.get(), 0, spatial_features_size_ * sizeof(float), stream_));

  if (!preprocess(input_pointcloud_msg_ptr, tf_buffer)) {
    RCLCPP_WARN(
      rclcpp::get_logger(config_.logger_name_.c_str()), "Fail to preprocess and skip to detect.");
    return false;
  }

  inference();
  postProcess(det_boxes3d);

  // Check the actual number of pillars after inference to avoid unnecessary synchronization.
  unsigned int num_pillars = 0;
  CHECK_CUDA_ERROR(
    cudaMemcpy(&num_pillars, num_voxels_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost));

  if (num_pillars >= config_.max_voxel_size_) {
    rclcpp::Clock clock{RCL_ROS_TIME};
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(config_.logger_name_.c_str()), clock, 1000,
      "The actual number of pillars (%u) exceeds its maximum value (%zu). "
      "Please considering increasing it since it may limit the detection performance.",
      num_pillars, config_.max_voxel_size_);
    is_num_pillars_within_range = false;
  }

  return true;
}

bool CenterPointTRT::preprocess(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & input_pointcloud_msg_ptr,
  const tf2_ros::Buffer & tf_buffer)
{
  bool is_success = vg_ptr_->enqueuePointCloud(input_pointcloud_msg_ptr, tf_buffer);
  if (!is_success) {
    return false;
  }

  const std::size_t count = vg_ptr_->generateSweepPoints(points_aux_d_.get(), stream_);
  const std::size_t random_offset = std::rand() % config_.cloud_capacity_;
  CHECK_CUDA_ERROR(shufflePoints_launch(
    points_aux_d_.get(), shuffle_indices_d_.get(), points_d_.get(), count, config_.cloud_capacity_,
    random_offset, stream_));

  CHECK_CUDA_ERROR(cudaMemsetAsync(num_voxels_d_.get(), 0, sizeof(unsigned int), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(voxels_buffer_d_.get(), 0, voxels_buffer_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(mask_d_.get(), 0, mask_size_ * sizeof(int), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(voxels_d_.get(), 0, voxels_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(coordinates_d_.get(), 0, coordinates_size_ * sizeof(int), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    num_points_per_voxel_d_.get(), 0, config_.max_voxel_size_ * sizeof(float), stream_));

  CHECK_CUDA_ERROR(generateVoxels_random_launch(
    points_d_.get(), config_.cloud_capacity_, config_.range_min_x_, config_.range_max_x_,
    config_.range_min_y_, config_.range_max_y_, config_.range_min_z_, config_.range_max_z_,
    config_.voxel_size_x_, config_.voxel_size_y_, config_.voxel_size_z_, config_.grid_size_y_,
    config_.grid_size_x_, mask_d_.get(), voxels_buffer_d_.get(), stream_));

  CHECK_CUDA_ERROR(generateBaseFeatures_launch(
    mask_d_.get(), voxels_buffer_d_.get(), config_.grid_size_y_, config_.grid_size_x_,
    config_.max_voxel_size_, num_voxels_d_.get(), voxels_d_.get(), num_points_per_voxel_d_.get(),
    coordinates_d_.get(), stream_));

  CHECK_CUDA_ERROR(generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_d_.get(),
    config_.max_voxel_size_, config_.voxel_size_x_, config_.voxel_size_y_, config_.voxel_size_z_,
    config_.range_min_x_, config_.range_min_y_, config_.range_min_z_, encoder_in_features_d_.get(),
    config_.encoder_in_feature_size_, stream_));

  return true;
}

void CenterPointTRT::inference()
{
  // pillar encoder network
  std::vector<void *> encoder_tensors = {encoder_in_features_d_.get(), pillar_features_d_.get()};
  encoder_trt_ptr_->setTensorsAddresses(encoder_tensors);
  encoder_trt_ptr_->enqueueV3(stream_);

  // scatter
  CHECK_CUDA_ERROR(scatterFeatures_launch(
    pillar_features_d_.get(), coordinates_d_.get(), num_voxels_d_.get(), config_.max_voxel_size_,
    config_.encoder_out_feature_size_, config_.grid_size_x_, config_.grid_size_y_,
    spatial_features_d_.get(), stream_));

  // head network
  std::vector<void *> head_tensors = {spatial_features_d_.get(), head_out_heatmap_d_.get(),
                                      head_out_offset_d_.get(),  head_out_z_d_.get(),
                                      head_out_dim_d_.get(),     head_out_rot_d_.get(),
                                      head_out_vel_d_.get()};
  head_trt_ptr_->setTensorsAddresses(head_tensors);

  head_trt_ptr_->enqueueV3(stream_);
}

void CenterPointTRT::postProcess(std::vector<Box3D> & det_boxes3d)
{
  CHECK_CUDA_ERROR(post_proc_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_));
  if (det_boxes3d.size() == 0) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(config_.logger_name_.c_str()), "No detected boxes.");
  }
}

bool CenterPointTRT::detectWithTTA(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & input_pointcloud_msg_ptr,
  const tf2_ros::Buffer & tf_buffer, std::vector<Box3D> & det_boxes3d,
  bool & is_num_pillars_within_range)
{
  if (!tta_processor_ || !tta_processor_->isEnabled()) {
    return detect(input_pointcloud_msg_ptr, tf_buffer, det_boxes3d, is_num_pillars_within_range);
  }

  is_num_pillars_within_range = true;

  // Clear GPU memory
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    encoder_in_features_d_.get(), 0, encoder_in_feature_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(spatial_features_d_.get(), 0, spatial_features_size_ * sizeof(float), stream_));

  // Preprocess to get point cloud data
  if (!preprocess(input_pointcloud_msg_ptr, tf_buffer)) {
    RCLCPP_WARN(
      rclcpp::get_logger(config_.logger_name_.c_str()), "Fail to preprocess and skip to detect.");
    return false;
  }

  // Generate TTA augmentations using GPU kernels
  // Get rotation angles from TTA processor
  std::vector<float> rotation_angles_rad;
  for (const auto & angle_deg : tta_processor_->getRotationAngles()) {
    rotation_angles_rad.push_back(angle_deg * M_PI / 180.0f);
  }

  int num_augmentations = tta_processor_->getNumAugmentations();

  // Copy rotation angles to GPU
  auto rotation_angles_d = cuda::make_unique<float[]>(num_augmentations);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    rotation_angles_d.get(), rotation_angles_rad.data(), num_augmentations * sizeof(float),
    cudaMemcpyHostToDevice, stream_));

  // Generate augmented point clouds using GPU kernel
  rotatePointsParallel_launch(
    points_d_.get(), config_.cloud_capacity_, config_.point_feature_size_, rotation_angles_d.get(),
    num_augmentations, tta_points_d_.get(), stream_);

  // Process each augmentation
  std::vector<std::vector<Box3D>> all_detections;
  inferenceTTA(num_augmentations, all_detections);

  // Merge results
  det_boxes3d = mergeTTADetections(num_augmentations, rotation_angles_rad, all_detections);
  return true;
}

void CenterPointTRT::inferenceTTA(
  const int num_augmentations, std::vector<std::vector<Box3D>> & all_detections)
{
  all_detections.clear();
  all_detections.resize(num_augmentations);

  for (int i = 0; i < num_augmentations; ++i) {
    // Clear GPU memory for this augmentation
    CHECK_CUDA_ERROR(cudaMemsetAsync(
      tta_encoder_features_d_.get() + i * encoder_in_feature_size_, 0,
      encoder_in_feature_size_ * sizeof(float), stream_));
    CHECK_CUDA_ERROR(cudaMemsetAsync(
      tta_spatial_features_d_.get() + i * spatial_features_size_, 0,
      spatial_features_size_ * sizeof(float), stream_));

    // Generate voxels for this augmentation
    CHECK_CUDA_ERROR(
      cudaMemsetAsync(tta_num_voxels_d_.get() + i, 0, sizeof(unsigned int), stream_));

    // Use the same voxel generation as original but with augmented points
    CHECK_CUDA_ERROR(generateVoxels_random_launch(
      tta_points_d_.get() + i * config_.cloud_capacity_ * config_.point_feature_size_,
      config_.cloud_capacity_, config_.range_min_x_, config_.range_max_x_, config_.range_min_y_,
      config_.range_max_y_, config_.range_min_z_, config_.range_max_z_, config_.voxel_size_x_,
      config_.voxel_size_y_, config_.voxel_size_z_, config_.grid_size_y_, config_.grid_size_x_,
      mask_d_.get(), voxels_buffer_d_.get(), stream_));

    CHECK_CUDA_ERROR(generateBaseFeatures_launch(
      mask_d_.get(), voxels_buffer_d_.get(), config_.grid_size_y_, config_.grid_size_x_,
      config_.max_voxel_size_, tta_num_voxels_d_.get() + i, tta_voxels_d_.get() + i * voxels_size_,
      tta_num_points_per_voxel_d_.get() + i * config_.max_voxel_size_,
      tta_coordinates_d_.get() + i * coordinates_size_, stream_));

    CHECK_CUDA_ERROR(generateFeatures_launch(
      tta_voxels_d_.get() + i * voxels_size_,
      tta_num_points_per_voxel_d_.get() + i * config_.max_voxel_size_,
      tta_coordinates_d_.get() + i * coordinates_size_, tta_num_voxels_d_.get() + i,
      config_.max_voxel_size_, config_.voxel_size_x_, config_.voxel_size_y_, config_.voxel_size_z_,
      config_.range_min_x_, config_.range_min_y_, config_.range_min_z_,
      tta_encoder_features_d_.get() + i * encoder_in_feature_size_,
      config_.encoder_in_feature_size_, stream_));

    // Run encoder inference
    std::vector<void *> encoder_tensors = {
      tta_encoder_features_d_.get() + i * encoder_in_feature_size_, pillar_features_d_.get()};
    encoder_trt_ptr_->setTensorsAddresses(encoder_tensors);
    encoder_trt_ptr_->enqueueV3(stream_);

    // Scatter features
    CHECK_CUDA_ERROR(scatterFeatures_launch(
      pillar_features_d_.get(), tta_coordinates_d_.get() + i * coordinates_size_,
      tta_num_voxels_d_.get() + i, config_.max_voxel_size_, config_.encoder_out_feature_size_,
      config_.grid_size_x_, config_.grid_size_y_,
      tta_spatial_features_d_.get() + i * spatial_features_size_, stream_));

    // Run head inference
    std::vector<void *> head_tensors = {
      tta_spatial_features_d_.get() + i * spatial_features_size_,
      head_out_heatmap_d_.get(),
      head_out_offset_d_.get(),
      head_out_z_d_.get(),
      head_out_dim_d_.get(),
      head_out_rot_d_.get(),
      head_out_vel_d_.get()};
    head_trt_ptr_->setTensorsAddresses(head_tensors);
    head_trt_ptr_->enqueueV3(stream_);

    // Post-process to get detections
    CHECK_CUDA_ERROR(post_proc_ptr_->generateDetectedBoxes3D_launch(
      head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(),
      head_out_dim_d_.get(), head_out_rot_d_.get(), head_out_vel_d_.get(), all_detections[i],
      stream_));
  }
}

std::vector<Box3D> CenterPointTRT::mergeTTADetections(
  const int num_augmentations, const std::vector<float> & rotation_angles_rad,
  const std::vector<std::vector<Box3D>> & all_detections)
{
  // Transform detections back to original coordinate system
  std::vector<std::vector<Box3D>> transformed_detections;
  transformed_detections.resize(num_augmentations);

  // Debug: Log original detections for each augmentation
  for (int i = 0; i < num_augmentations; ++i) {
    float angle_deg =
      -rotation_angles_rad[i] * 180.0f / M_PI;  // Convert back to degrees for display
    RCLCPP_INFO(
      rclcpp::get_logger(config_.logger_name_.c_str()),
      "TTA Augmentation %d (%.1f° rotation): %zu detections", i, angle_deg,
      all_detections[i].size());

    // Log first few detections for comparison
    for (size_t j = 0; j < std::min(all_detections[i].size(), size_t(3)); ++j) {
      const auto & box = all_detections[i][j];
      RCLCPP_INFO(
        rclcpp::get_logger(config_.logger_name_.c_str()),
        "  Before transform - Box %zu: pos(%.2f, %.2f, %.2f) size(%.2f, %.2f, %.2f) score: %.3f", j,
        box.x, box.y, box.z, box.length, box.width, box.height, box.score);
    }
  }

  for (int i = 0; i < num_augmentations; ++i) {
    // Create inverse transformation matrix for this rotation
    Eigen::Matrix4f inverse_transform = Eigen::Matrix4f::Identity();
    float angle = -rotation_angles_rad[i];  // Inverse rotation
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);

    inverse_transform(0, 0) = cos_angle;
    inverse_transform(0, 1) = -sin_angle;
    inverse_transform(1, 0) = sin_angle;
    inverse_transform(1, 1) = cos_angle;

    // Transform detections back to original coordinate system
    transformed_detections[i] =
      tta_processor_->transformBoxes(all_detections[i], inverse_transform);
  }

  // Debug: Log transformed detections for comparison
  for (int i = 0; i < num_augmentations; ++i) {
    float angle_deg = -rotation_angles_rad[i] * 180.0f / M_PI;
    RCLCPP_INFO(
      rclcpp::get_logger(config_.logger_name_.c_str()),
      "TTA Augmentation %d (%.1f° rotation): %zu detections after transform", i, angle_deg,
      transformed_detections[i].size());

    // Log first few detections after transformation
    for (size_t j = 0; j < std::min(transformed_detections[i].size(), size_t(3)); ++j) {
      const auto & box = transformed_detections[i][j];
      RCLCPP_INFO(
        rclcpp::get_logger(config_.logger_name_.c_str()),
        "  After transform - Box %zu: pos(%.2f, %.2f, %.2f) size(%.2f, %.2f, %.2f) score: %.3f", j,
        box.x, box.y, box.z, box.length, box.width, box.height, box.score);
    }
  }

  // Merge all detections using TTA processor
  std::vector<TTAResult> tta_results;
  tta_results.resize(num_augmentations);
  for (int i = 0; i < num_augmentations; ++i) {
    tta_results[i].detected_boxes = transformed_detections[i];
  }

  std::vector<Box3D> merged_results = tta_processor_->mergeTTAResults(tta_results);

  // Debug: Log final merged results
  RCLCPP_INFO(
    rclcpp::get_logger(config_.logger_name_.c_str()), "TTA Final merged results: %zu detections",
    merged_results.size());

  for (size_t i = 0; i < std::min(merged_results.size(), size_t(5)); ++i) {
    const auto & box = merged_results[i];
    RCLCPP_INFO(
      rclcpp::get_logger(config_.logger_name_.c_str()),
      "  Final Box %zu: pos(%.2f, %.2f, %.2f) size(%.2f, %.2f, %.2f) score: %.3f", i, box.x, box.y,
      box.z, box.length, box.width, box.height, box.score);
  }

  return merged_results;
}

}  // namespace autoware::lidar_centerpoint
