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

#include "autoware/camera_streampetr/network/network.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

using autoware::tensorrt_common::TrtCommonConfig;
using Label = autoware_perception_msgs::msg::ObjectClassification;

std::uint8_t get_semantic_type(const std::string & class_name)
{
  static const std::unordered_map<std::string, std::uint8_t> class_mapping = {
    {"CAR", Label::CAR},
    {"TRUCK", Label::TRUCK},
    {"BUS", Label::BUS},
    {"TRAILER", Label::TRAILER},
    {"MOTORCYCLE", Label::MOTORCYCLE},
    {"BICYCLE", Label::BICYCLE},
    {"PEDESTRIAN", Label::PEDESTRIAN},
    {"TRAFFIC_CONE", Label::HAZARD},
    {"BARRIER", Label::HAZARD}};

  const auto it = class_mapping.find(class_name);
  return (it != class_mapping.end()) ? it->second : Label::UNKNOWN;
}

autoware_perception_msgs::msg::DetectedObject StreamPetrNetwork::bbox_to_ros_msg(const Box3D & bbox)
{
  // cx, cy, cz, w, l, h, rot, vx, vy
  autoware_perception_msgs::msg::DetectedObject object;
  object.kinematics.pose_with_covariance.pose.position.x = bbox.x;
  object.kinematics.pose_with_covariance.pose.position.y = bbox.y;
  object.kinematics.pose_with_covariance.pose.position.z = bbox.z;
  object.shape.dimensions.x = bbox.width;
  object.shape.dimensions.y = bbox.length;
  object.shape.dimensions.z = bbox.height;
  const double yaw = bbox.yaw;
  object.kinematics.pose_with_covariance.pose.orientation.w = cos(yaw * 0.5);
  object.kinematics.pose_with_covariance.pose.orientation.x = 0;
  object.kinematics.pose_with_covariance.pose.orientation.y = 0;
  object.kinematics.pose_with_covariance.pose.orientation.z = sin(yaw * 0.5);

  object.existence_probability = bbox.score;
  object.kinematics.has_position_covariance = false;
  object.kinematics.has_twist = false;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  autoware_perception_msgs::msg::ObjectClassification classification;
  // The detection score goes here as well, matching the other Autoware detectors.
  classification.probability = bbox.score;
  classification.label = get_semantic_type(config_.class_names[bbox.label]);
  object.classification.push_back(classification);
  return object;
}

namespace
{

bool should_set_layer_to_fp32(nvinfer1::ILayer * layer, const std::string & layer_name_lower)
{
  // Check layer name for sigmoid/softmax keywords
  if (
    layer_name_lower.find("sigmoid") != std::string::npos ||
    layer_name_lower.find("softmax") != std::string::npos) {
    return true;
  }

  // Check layer type for activation or softmax layers
  if (layer->getType() == nvinfer1::LayerType::kACTIVATION) {
    auto * activation_layer = static_cast<nvinfer1::IActivationLayer *>(layer);
    return activation_layer->getActivationType() == nvinfer1::ActivationType::kSIGMOID;
  }

  return layer->getType() == nvinfer1::LayerType::kSOFTMAX;
}

}  // anonymous namespace

void set_sigmoid_and_softmax_layers_to_fp32(std::shared_ptr<nvinfer1::INetworkDefinition> network)
{
  for (int i = 0; i < network->getNbLayers(); ++i) {
    auto * layer = network->getLayer(i);
    std::string layer_name = layer->getName();

    // Convert to lowercase for case-insensitive comparison
    std::transform(layer_name.begin(), layer_name.end(), layer_name.begin(), ::tolower);

    if (should_set_layer_to_fp32(layer, layer_name)) {
      layer->setPrecision(nvinfer1::DataType::kFLOAT);
    }
  }
}

StreamPetrNetwork::StreamPetrNetwork(const NetworkConfig & config) : config_(config)
{
  incremental_backbone_ = !config_.onnx_backbone_batch1_path.empty();
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  initialize_networks();
  setup_engines();
  setup_bindings();
  validate_bindings();
  alias_shared_bindings();
  initialize_memory_and_profiling();
  configure_nms_if_needed();
}

void StreamPetrNetwork::initialize_networks()
{
  // Initialize TrtCommon configurations
  // The batch-1 variant replaces the batch backbone; loading both would double activation memory.
  const std::string & backbone_onnx =
    incremental_backbone_ ? config_.onnx_backbone_batch1_path : config_.onnx_backbone_path;
  auto backbone_config = tensorrt_common::TrtCommonConfig(
    backbone_onnx, config_.trt_precision,
    incremental_backbone_ ? std::string("") : config_.engine_backbone_path, config_.workspace_size);
  auto pts_head_config = tensorrt_common::TrtCommonConfig(
    config_.onnx_head_path, config_.trt_precision, config_.engine_head_path,
    config_.workspace_size);
  auto pos_embed_config = tensorrt_common::TrtCommonConfig(
    config_.onnx_position_embedding_path, config_.trt_precision,
    config_.engine_position_embedding_path, config_.workspace_size);

  // Initialize TrtCommon instances
  backbone_ = std::make_unique<SubNetwork>(backbone_config);
  pts_head_ = std::make_unique<SubNetwork>(pts_head_config);
  pos_embed_ = std::make_unique<SubNetwork>(pos_embed_config);

  // Apply FP32 precision for stability if using fp16
  if (config_.trt_precision == "fp16") {
    auto logger = rclcpp::get_logger(config_.logger_name.c_str());
    RCLCPP_INFO(logger, "Setting sigmoid and softmax layers to FP32 precision for stability");
    set_sigmoid_and_softmax_layers_to_fp32(pts_head_->getNetwork());
  }
}

void StreamPetrNetwork::setup_engines()
{
  if (!backbone_->setup()) {
    throw std::runtime_error("Failed to setup backbone TRT engine.");
  }
  if (!pts_head_->setup()) {
    throw std::runtime_error("Failed to setup pts_head TRT engine.");
  }
  if (!pos_embed_->setup()) {
    throw std::runtime_error("Failed to setup pos_embed TRT engine.");
  }
}

void StreamPetrNetwork::setup_bindings()
{
  auto logger = rclcpp::get_logger(config_.logger_name.c_str());

  if (!backbone_->set_bindings(logger)) {
    throw std::runtime_error("Failed to setup backbone TRT bindings.");
  }
  if (!pts_head_->set_bindings(logger)) {
    throw std::runtime_error("Failed to setup pts_head TRT bindings.");
  }
  if (!pos_embed_->set_bindings(logger)) {
    throw std::runtime_error("Failed to setup pos_embed TRT bindings.");
  }
}

namespace
{
void expect_volume(
  const SubNetwork & network, const std::string & binding_name, const int64_t expected,
  const std::string & derived_from)
{
  const int64_t actual = network.binding(binding_name)->volume;
  if (actual != expected) {
    throw std::runtime_error(
      "Binding '" + binding_name + "' holds " + std::to_string(actual) + " elements but " +
      derived_from + " implies " + std::to_string(expected) +
      ". The engine and the parameter file disagree.");
  }
}
}  // namespace

void StreamPetrNetwork::validate_bindings() const
{
  // Fail early with a readable error if the engine is missing an expected binding.
  for (const auto & name : {"img", "img_feats"}) {
    backbone_->binding(name);
  }
  for (const auto & name :
       {"x", "pos_embed", "cone", "data_ego_pose", "data_ego_pose_inv", "all_cls_scores",
        "all_bbox_preds", "pre_memory_timestamp", "post_memory_timestamp", "pre_memory_embedding",
        "post_memory_embedding", "pre_memory_reference_point", "post_memory_reference_point",
        "pre_memory_egopose", "post_memory_egopose", "pre_memory_velo", "post_memory_velo"}) {
    pts_head_->binding(name);
  }
  for (const auto & name : {"img_metas_pad", "intrinsics", "img2lidar", "pos_embed", "cone"}) {
    pos_embed_->binding(name);
  }

  const int64_t num_classes = static_cast<int64_t>(config_.class_names.size());
  const int64_t num_proposals = config_.num_proposals;

  const int64_t backbone_cameras = incremental_backbone_ ? 1 : config_.image_num;
  expect_volume(
    *backbone_, "img", backbone_cameras * 3 * config_.image_height * config_.image_width,
    "model_params.rois_number x 3 x input_image_height x input_image_width");
  expect_volume(
    *pts_head_, "all_cls_scores", num_classes * num_proposals,
    "model_params.class_names x model_params.num_proposals");
  expect_volume(
    *pts_head_, "pre_memory_timestamp", config_.pre_memory_length,
    "model_params.pre_memory_length");
  expect_volume(
    *pts_head_, "post_memory_timestamp", config_.post_memory_length,
    "model_params.post_memory_length");

  // generate_boxes3d_kernel indexes box_output up to point_idx + 7 * num_proposals.
  const int64_t bbox_volume = pts_head_->binding("all_bbox_preds")->volume;
  if (bbox_volume % num_proposals != 0 || bbox_volume / num_proposals < 8) {
    throw std::runtime_error(
      "Binding 'all_bbox_preds' holds " + std::to_string(bbox_volume) +
      " elements, which is not at least 8 channels of model_params.num_proposals (" +
      std::to_string(num_proposals) + ").");
  }

  // The postprocessing kernel reads detection_range[0..5] unconditionally.
  if (config_.detection_range.size() < 6) {
    throw std::runtime_error(
      "model_params.detection_range needs 6 values [-x,-y,-z,x,y,z] but has " +
      std::to_string(config_.detection_range.size()) + ".");
  }
}

void StreamPetrNetwork::alias_shared_bindings()
{
  if (!incremental_backbone_) {
    // Alias "x" onto "img_feats" so the feature tensor is never copied device-to-device.
    pts_head_->alias_binding("x", backbone_->binding("img_feats"));
    return;
  }

  // Cameras write one buffer while the head reads the other: the feature tensor is
  // double-buffered.
  const auto & x = pts_head_->binding("x");
  feature_slot_bytes_ = static_cast<std::size_t>(x->nbytes()) / config_.image_num;
  if (backbone_->binding("img_feats")->nbytes() != feature_slot_bytes_) {
    throw std::runtime_error(
      "The batch-1 backbone's img_feats does not match one camera slot of the head's x input.");
  }
  for (auto & buffer : feature_buffers_) {
    buffer = std::make_shared<Tensor>("img_feats_buffer", x->dim, x->dtype);
  }
  pts_head_->alias_binding("x", feature_buffers_[1]);  // read buffer before the first swap

  extract_done_events_.resize(config_.image_num);
  extract_done_valid_.assign(config_.image_num, false);
  for (auto & event : extract_done_events_) {
    CHECK_CUDA_ERROR(cudaEventCreateWithFlags(&event, cudaEventDisableTiming));
  }
}

std::shared_ptr<Tensor> StreamPetrNetwork::create_image_input_tensor()
{
  if (!incremental_backbone_) {
    return backbone_->binding("img");
  }
  // Standalone tensor: the engine's own binding has only a single camera slot.
  return std::make_shared<Tensor>(
    "image_input",
    nvinfer1::Dims{5, {1, config_.image_num, 3, config_.image_height, config_.image_width}},
    nvinfer1::DataType::kFLOAT);
}

void StreamPetrNetwork::extract_features(float * slot_ptr, int camera_id, cudaStream_t stream)
{
  // Address setting and enqueue must not interleave across camera callback threads.
  std::lock_guard<std::mutex> lock(backbone_mutex_);
  backbone_->setTensorAddress("img", slot_ptr);
  backbone_->setTensorAddress(
    "img_feats", static_cast<char *>(feature_buffers_[feature_write_index_]->ptr) +
                   static_cast<std::size_t>(camera_id) * feature_slot_bytes_);
  if (!backbone_->enqueueV3(stream)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(config_.logger_name.c_str()),
      "Batch-1 backbone enqueue failed for camera %d", camera_id);
    return;
  }
  CHECK_CUDA_ERROR(cudaEventRecord(extract_done_events_[camera_id], stream));
  extract_done_valid_[camera_id] = true;
}

void StreamPetrNetwork::initialize_memory_and_profiling()
{
  mem_.init(stream_, config_.pre_memory_length, config_.post_memory_length);
  mem_.pre_buf = static_cast<float *>(pts_head_->binding("pre_memory_timestamp")->ptr);
  mem_.post_buf = static_cast<float *>(pts_head_->binding("post_memory_timestamp")->ptr);

  // cudaMalloc does not zero: without this the first inference would read garbage temporal state.
  wipe_memory();

  // events for measurement
  dur_backbone_ = std::make_unique<Duration>("backbone");
  dur_ptshead_ = std::make_unique<Duration>("ptshead");
  dur_pos_embed_ = std::make_unique<Duration>("pos_embed");

  postprocess_cuda_ = std::make_unique<PostprocessCuda>(
    PostProcessingConfig(
      config_.class_names.size(), config_.circle_nms_dist_threshold, config_.confidence_thresholds,
      config_.yaw_norm_thresholds, config_.num_proposals, config_.detection_range),
    stream_);
}

void StreamPetrNetwork::configure_nms_if_needed()
{
  if (config_.iou_threshold > 0.0) {
    NMSParams p;
    p.search_distance_2d_ = config_.search_distance_2d;
    p.iou_threshold_ = config_.iou_threshold;
    iou_bev_nms_.set_parameters(p);
  }
}

void StreamPetrNetwork::wipe_memory()
{
  pts_head_->binding("pre_memory_embedding")->initialize_to_zeros(stream_);
  pts_head_->binding("pre_memory_reference_point")->initialize_to_zeros(stream_);
  pts_head_->binding("pre_memory_egopose")->initialize_to_zeros(stream_);
  pts_head_->binding("pre_memory_velo")->initialize_to_zeros(stream_);
  // The timestamps in mem_buf are part of the same temporal state and must be dropped with it.
  mem_.clear();
  mem_.step_reset();
}

void StreamPetrNetwork::inference_detector(
  const InferenceInputs & inputs, SubNetworkTimings & subnetwork_timings)
{
  if (!is_inference_initialized_) {
    initialize_position_embedding(inputs);
    is_inference_initialized_ = true;
  }

  if (incremental_backbone_) {
    // Swap: the head reads what the cameras just wrote; they start filling the other buffer.
    {
      std::lock_guard<std::mutex> lock(backbone_mutex_);
      head_graph_slot_ = feature_write_index_;
      feature_write_index_ ^= 1;
      for (std::size_t camera_id = 0; camera_id < extract_done_events_.size(); ++camera_id) {
        if (extract_done_valid_[camera_id]) {
          CHECK_CUDA_ERROR(cudaStreamWaitEvent(stream_, extract_done_events_[camera_id], 0));
        }
      }
    }
    pts_head_->alias_binding("x", feature_buffers_[head_graph_slot_]);
    // Backbone time is spent per camera; there is no per-frame batch pass to report.
    dur_backbone_->mark_begin(stream_);
    dur_backbone_->mark_end(stream_);
  } else {
    execute_backbone(inputs);
  }
  execute_pts_head(inputs);

  // Without this sync the caller's inference time would only measure the enqueueing.
  cudaStreamSynchronize(stream_);

  subnetwork_timings.backbone_ms = dur_backbone_->elapsed();
  subnetwork_timings.ptshead_ms = dur_ptshead_->elapsed();
  subnetwork_timings.pos_embed_ms = dur_pos_embed_->elapsed();
}

void StreamPetrNetwork::initialize_position_embedding(const InferenceInputs & inputs)
{
  pos_embed_->binding("img_metas_pad")->load_from_vector(inputs.img_metas_pad);
  pos_embed_->binding("intrinsics")->load_from_vector(inputs.intrinsics);
  pos_embed_->binding("img2lidar")->load_from_vector(inputs.img2lidar);

  dur_pos_embed_->mark_begin(stream_);
  pos_embed_->enqueueV3(stream_);
  dur_pos_embed_->mark_end(stream_);

  // Only ever executed once, so these two copies are not worth aliasing away.
  pts_head_->binding("pos_embed")->copy(pos_embed_->binding("pos_embed"), stream_);
  pts_head_->binding("cone")->copy(pos_embed_->binding("cone"), stream_);
}

void StreamPetrNetwork::execute_backbone(const InferenceInputs & inputs)
{
  if (inputs.imgs->ptr != backbone_->binding("img")->ptr) {
    throw std::runtime_error(
      "The preprocessed image tensor is not the backbone's 'img' binding; the zero-copy path was "
      "not wired up.");
  }

  dur_backbone_->mark_begin(stream_);
  backbone_->enqueueV3(stream_);
  dur_backbone_->mark_end(stream_);
}

// The memory step kernels stay outside the graph: a captured graph would freeze their
// per-frame timestamp argument.
void StreamPetrNetwork::enqueue_pts_head_graph()
{
  cudaGraphExec_t & head_graph_ = head_graphs_[head_graph_slot_];
  bool & head_warmed_up_ = head_warmed_[head_graph_slot_];

  if (head_graph_ != nullptr) {
    CHECK_CUDA_ERROR(cudaGraphLaunch(head_graph_, stream_));
    return;
  }

  if (head_graph_unusable_ || !head_warmed_up_) {
    pts_head_->enqueueV3(stream_);
    head_warmed_up_ = true;
    return;
  }

  // Thread-local capture mode keeps the camera streams' concurrent work out of the capture.
  cudaGraph_t graph = nullptr;
  bool captured = cudaStreamBeginCapture(stream_, cudaStreamCaptureModeThreadLocal) == cudaSuccess;
  if (captured) {
    captured = pts_head_->enqueueV3(stream_);
    // EndCapture must run even after a failed enqueue, or the stream stays in capture mode.
    captured = (cudaStreamEndCapture(stream_, &graph) == cudaSuccess) && captured;
  }
  if (captured && graph != nullptr) {
    captured = cudaGraphInstantiate(&head_graph_, graph, 0) == cudaSuccess;
  }
  if (graph != nullptr) {
    cudaGraphDestroy(graph);
  }

  if (captured && head_graph_ != nullptr) {
    RCLCPP_INFO(
      rclcpp::get_logger(config_.logger_name.c_str()), "pts_head captured as a CUDA graph");
    CHECK_CUDA_ERROR(cudaGraphLaunch(head_graph_, stream_));
    return;
  }

  head_graph_unusable_ = true;
  if (head_graph_ != nullptr) {
    cudaGraphExecDestroy(head_graph_);
    head_graph_ = nullptr;
  }
  // Clear any sticky capture error so CHECK_CUDA_ERROR on later calls does not trip on it.
  cudaGetLastError();
  RCLCPP_WARN(
    rclcpp::get_logger(config_.logger_name.c_str()),
    "pts_head CUDA graph capture failed; falling back to per-kernel launches");
  pts_head_->enqueueV3(stream_);
}

void StreamPetrNetwork::execute_pts_head(const InferenceInputs & inputs)
{
  // Async on stream_: the synchronous overload's legacy-default-stream copy would wait for the
  // camera streams to drain. Stream ordering keeps these visible to the enqueueV3 below.
  pts_head_->binding("data_ego_pose")->load_from_vector_async(inputs.ego_pose, stream_);
  pts_head_->binding("data_ego_pose_inv")->load_from_vector_async(inputs.ego_pose_inv, stream_);

  dur_ptshead_->mark_begin(stream_);

  mem_.step_pre(inputs.stamp);
  enqueue_pts_head_graph();
  mem_.step_post(inputs.stamp);

  if (config_.use_temporal) {
    // Tensor::copy moves the destination's size, deliberately keeping only pre_memory_length
    // entries.
    pts_head_->binding("pre_memory_embedding")
      ->copy(pts_head_->binding("post_memory_embedding"), stream_);
    pts_head_->binding("pre_memory_reference_point")
      ->copy(pts_head_->binding("post_memory_reference_point"), stream_);
    pts_head_->binding("pre_memory_egopose")
      ->copy(pts_head_->binding("post_memory_egopose"), stream_);
    pts_head_->binding("pre_memory_velo")->copy(pts_head_->binding("post_memory_velo"), stream_);
  } else {
    wipe_memory();
  }
  dur_ptshead_->mark_end(stream_);
}

void StreamPetrNetwork::postprocess(
  std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects)
{
  std::vector<Box3D> det_boxes3d;
  CHECK_CUDA_ERROR(postprocess_cuda_->generate_detected_boxes3d_launch(
    static_cast<const float *>(pts_head_->binding("all_cls_scores")->ptr),
    static_cast<const float *>(pts_head_->binding("all_bbox_preds")->ptr), det_boxes3d, stream_));

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  for (size_t i = 0; i < det_boxes3d.size(); ++i) {
    raw_objects.push_back(this->bbox_to_ros_msg(det_boxes3d[i]));
  }

  if (config_.iou_threshold > 0.0) {
    iou_bev_nms_.apply(raw_objects, output_objects);
  } else {
    output_objects = std::move(raw_objects);
  }
}

StreamPetrNetwork::~StreamPetrNetwork()
{
  for (auto & event : extract_done_events_) {
    cudaEventDestroy(event);
  }
  for (auto & graph : head_graphs_) {
    if (graph != nullptr) {
      cudaGraphExecDestroy(graph);
      graph = nullptr;
    }
  }
  if (stream_) {
    // Destroying a stream with work still in flight is undefined behaviour.
    cudaStreamSynchronize(stream_);
    mem_.release();
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

}  // namespace autoware::camera_streampetr
