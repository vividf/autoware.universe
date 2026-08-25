// Copyright 2025 TIER IV
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>

// From NVIDIA/DL4AGX
#include "autoware/camera_streampetr/network/memory.cuh"

#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
// From NVIDIA/DL4AGX

#include "autoware/camera_streampetr/cuda_utils.hpp"
#include "autoware/camera_streampetr/postprocess/non_maximum_suppression.hpp"
#include "autoware/camera_streampetr/postprocess/postprocess_kernel.hpp"
#include "autoware/camera_streampetr/utils.hpp"

// TensorRT Common
#include "autoware/tensorrt_common/tensorrt_common.hpp"
#include "autoware/tensorrt_common/utils.hpp"

// CUDA utilities
#include <autoware/cuda_utils/cuda_utils.hpp>

namespace autoware::camera_streampetr
{

using cuda::Tensor;
using nvinfer1::DataType;
using nvinfer1::Dims;

// Use tensorrt_common components
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

class SubNetwork : public TrtCommon
{
public:
  std::unordered_map<std::string, std::shared_ptr<Tensor>> bindings;

  using TrtCommon::TrtCommon;
  virtual ~SubNetwork() = default;

  /// Named binding lookup; throws on unknown names instead of operator[]'s silent null insert.
  const std::shared_ptr<Tensor> & binding(const std::string & name) const
  {
    const auto it = bindings.find(name);
    if (it == bindings.end()) {
      std::string known;
      for (const auto & [binding_name, tensor] : bindings) {
        known += (known.empty() ? "" : ", ") + binding_name;
      }
      throw std::runtime_error(
        "TensorRT engine has no binding named '" + name + "'. Available bindings: " + known + ".");
    }
    return it->second;
  }

  /// Points the engine at a caller-owned buffer; the caller must keep it alive for this engine.
  void alias_binding(const std::string & name, const std::shared_ptr<Tensor> & tensor)
  {
    const auto & current = binding(name);
    if (current->nbytes() != tensor->nbytes()) {
      throw std::runtime_error(
        "Cannot alias binding '" + name + "' (" + std::to_string(current->nbytes()) +
        " bytes) to tensor '" + tensor->name + "' (" + std::to_string(tensor->nbytes()) +
        " bytes): sizes differ.");
    }
    bindings[name] = tensor;
    setTensorAddress(name.c_str(), tensor->ptr);
  }

  bool set_bindings(const rclcpp::Logger & logger)
  {
    for (int n = 0; n < getNbIOTensors(); n++) {
      std::string name = getIOTensorName(n);
      Dims d = getTensorShape(name.c_str());
      auto dtype_opt = getTensorDataType(name.c_str());
      if (!dtype_opt.has_value()) {
        RCLCPP_WARN(logger, "Warning: Could not get data type for tensor: %s", name.c_str());
        return false;
      }
      DataType dtype = dtype_opt.value();
      bindings[name] = std::make_shared<Tensor>(name, d, dtype);
      bindings[name]->iomode = getTensorIOMode(name.c_str());

      std::stringstream ss;
      ss << *(bindings[name]);
      const std::string & str = ss.str();
      RCLCPP_INFO(logger, "%s", str.c_str());

      setTensorAddress(name.c_str(), bindings[name]->ptr);
    }
    return true;
  }
};

class Duration
{
  // CUDA events for timing
  cudaEvent_t begin_event_, end_event_;
  std::string layer_name_;

public:
  explicit Duration(const std::string & name) : layer_name_(name)
  {
    CHECK_CUDA_ERROR(cudaEventCreate(&begin_event_));
    CHECK_CUDA_ERROR(cudaEventCreate(&end_event_));
  }

  ~Duration()
  {
    cudaEventDestroy(begin_event_);
    cudaEventDestroy(end_event_);
  }

  const std::string & name() const { return layer_name_; }

  void mark_begin(cudaStream_t stream) { CHECK_CUDA_ERROR(cudaEventRecord(begin_event_, stream)); }

  void mark_end(cudaStream_t stream) { CHECK_CUDA_ERROR(cudaEventRecord(end_event_, stream)); }

  float elapsed()
  {
    // Without this wait cudaEventElapsedTime returns cudaErrorNotReady and leaves the output
    // parameter untouched.
    CHECK_CUDA_ERROR(cudaEventSynchronize(end_event_));

    float elapsed_ms = 0.0f;
    CHECK_CUDA_ERROR(cudaEventElapsedTime(&elapsed_ms, begin_event_, end_event_));
    return elapsed_ms;
  }
};  // class Duration

struct NetworkConfig
{
  // Logging
  std::string logger_name = "camera_streampetr";

  // Model parameters
  bool use_temporal = true;
  double search_distance_2d = 0.0;
  double circle_nms_dist_threshold = 0.0;
  double iou_threshold = 0.0;
  std::vector<double> confidence_thresholds;
  std::vector<std::string> class_names;
  int32_t num_proposals = 0;
  std::vector<double> yaw_norm_thresholds;
  std::vector<float> detection_range;
  int pre_memory_length = 0;
  int post_memory_length = 0;

  int image_height = 0;
  int image_width = 0;
  int image_num = 0;

  uint64_t workspace_size = 0;
  std::string trt_precision;

  // Engine paths
  std::string onnx_backbone_path;
  std::string onnx_head_path;
  std::string onnx_position_embedding_path;

  std::string engine_backbone_path = "";
  std::string engine_head_path = "";
  std::string engine_position_embedding_path = "";
};

struct InferenceInputs
{
  std::shared_ptr<Tensor> imgs;
  std::vector<float> ego_pose;
  std::vector<float> ego_pose_inv;
  std::vector<float> img_metas_pad;
  std::vector<float> intrinsics;
  std::vector<float> img2lidar;
  float stamp;
};

// GPU execution time of each subnetwork, measured with CUDA events.
struct SubNetworkTimings
{
  float backbone_ms{0.0f};
  float ptshead_ms{0.0f};
  float pos_embed_ms{0.0f};
};

class StreamPetrNetwork
{
public:
  explicit StreamPetrNetwork(const NetworkConfig & config);

  ~StreamPetrNetwork();

  // Runs the model only; returns with the stream synchronized.
  void inference_detector(const InferenceInputs & inputs, SubNetworkTimings & subnetwork_timings);

  // Reads only the head's output bindings, so the camera store may be unfrozen before calling.
  void postprocess(std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects);

  /// Drops the temporal state. Safe to call before the first inference.
  void wipe_memory();

  /// The backbone's own "img" input buffer; preprocessing writes straight into it.
  const std::shared_ptr<Tensor> & image_input_binding() const;

  /// Stream every inference kernel is enqueued on.
  cudaStream_t stream() const { return stream_; }

private:
  autoware_perception_msgs::msg::DetectedObject bbox_to_ros_msg(const Box3D & bbox);

  // Helper methods for constructor
  void initialize_networks();
  void setup_engines();
  void setup_bindings();
  void validate_bindings() const;
  void alias_shared_bindings();
  void initialize_memory_and_profiling();
  void configure_nms_if_needed();

  // Helper methods for inference_detector
  void initialize_position_embedding(const InferenceInputs & inputs);
  void execute_backbone(const InferenceInputs & inputs);
  void execute_pts_head(const InferenceInputs & inputs);
  void enqueue_pts_head_graph();

  NetworkConfig config_;
  std::unique_ptr<SubNetwork> backbone_;
  std::unique_ptr<SubNetwork> pts_head_;
  std::unique_ptr<SubNetwork> pos_embed_;

  std::unique_ptr<Duration> dur_backbone_;
  std::unique_ptr<Duration> dur_ptshead_;
  std::unique_ptr<Duration> dur_pos_embed_;

  std::unique_ptr<PostprocessCuda> postprocess_cuda_;
  NonMaximumSuppression iou_bev_nms_;

  bool is_inference_initialized_ = false;
  Memory mem_;
  cudaStream_t stream_;

  // The head's enqueueV3 is captured into a CUDA graph after a warmup enqueue; any capture
  // failure falls back to plain enqueueV3 for good. Capturing is safe because every address the
  // head reads or writes is fixed at construction.
  cudaGraphExec_t head_graph_{nullptr};
  bool head_warmed_up_{false};
  bool head_graph_unusable_{false};
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_
