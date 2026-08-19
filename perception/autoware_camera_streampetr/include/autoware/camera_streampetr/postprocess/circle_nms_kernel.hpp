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

#ifndef AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_

#include "autoware/camera_streampetr/utils.hpp"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace autoware::camera_streampetr
{
// cspell: ignore divup
/// Number of boxes each NMS bitmask row covers.
constexpr std::size_t CIRCLE_NMS_BOXES_PER_BLOCK = 16;

/// Worst-case element count of the workspace circle_nms() needs for `num_boxes3d` boxes.
inline std::size_t circle_nms_workspace_size(const std::size_t num_boxes3d)
{
  const std::size_t col_blocks =
    (num_boxes3d + CIRCLE_NMS_BOXES_PER_BLOCK - 1) / CIRCLE_NMS_BOXES_PER_BLOCK;
  return num_boxes3d * col_blocks;
}

// Non-maximum suppression (NMS) uses the distance on the xy plane instead of
// intersection over union (IoU) to suppress overlapped objects.
//
// All buffers are caller-owned so that nothing is allocated on the per-frame path:
//   boxes3d   [in]  num_boxes3d boxes, sorted by descending score
//   keep_mask [out] num_boxes3d flags
//   workspace [tmp] at least circle_nms_workspace_size(num_boxes3d) elements
std::size_t circle_nms(
  const Box3D * boxes3d, std::size_t num_boxes3d, float distance_threshold, bool * keep_mask,
  std::uint64_t * workspace, cudaStream_t stream);

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
