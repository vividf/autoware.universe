// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_
#define AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_

// Runtime precompute of the BEVFusion sparse-encoder down-sample rulebooks for the
// trainStation/DDS-stripped engine (see AWML deploy flag ``spconv_remove_trainstation`` and
// BEVFusion_spconv_DDS_optimization.md, Slice 2/2b).
//
// The optimized sparse ONNX has its 4 down-sample ``GetIndicePairsImplicitGemm`` nodes removed and
// their outputs exposed as graph inputs. Those rulebooks depend only on voxel geometry, so they are
// computed here from the voxel coordinates and bound to the engine before ``enqueueV3`` — removing
// the in-graph ``DeviceToShapeHostCopy`` syncs that produced the ``[trainStationN]`` segments.
//
// Two code paths produce identical rulebooks (selectable for A/B verification):
//
//   * Reference path (env ``BEVFUSION_RULEBOOK_REFERENCE=1``): one ``get_indice_pairs_implicit_gemm``
//     call per stage. Each call does an internal device-to-host copy of its active-output count
//     (spconv ``unique_hash`` -> ``.cpu()``) which is needed *within the same call* to size stage-2.
//     With 4 stages this is 4 blocking D2H syncs, serialized by the stage cascade.
//
//   * Device-count path (default): the cascade is run entirely on the GPU with each stage's active
//     count kept in a device buffer (we replicate spconv's ``unique`` step without the ``.cpu()`` --
//     see sparse_rulebook_precompute.cu). All 4 counts are copied to the host in a *single* D2H, then
//     the tight rulebooks are built. 4 syncs -> 1. Because the down-sample geometry is deterministic
//     (strides come from the ONNX metadata), inter-stage inputs are processed up to the static upper
//     bound ``out_indices_num_limit_`` with out-of-range sentinel padding, so no per-stage host count
//     is needed to chain the stages.
//
// This mirrors, per down-sample stage, the non-subm path of the GetIndicePairsImplicitGemm plugin
// (autoware_tensorrt_plugins/src/get_indices_pairs_implicit_gemm_plugin.cpp::enqueue) and the
// validated Python reference (AWML deployment/.../pipelines/sparse_rulebook_precompute.py).

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::bevfusion
{

using autoware::cuda_utils::CudaUniquePtr;

// One stride>1 (down-sampling) sparse convolution whose rulebook is precomputed.
struct SparseDownsampleStage
{
  // ONNX graph-input base name (e.g. ``rulebook/l1``); the 4 bound tensors are
  // ``<onnx_base>/{out_indices,pair_fwd,pair_mask,mask_argsort}`` — matches
  // export/sparse_trainstation_transform.py:rulebook_input_name().
  std::string onnx_base;
  std::vector<int> ksize;          // e.g. {3,3,3} ; conv_out {1,1,3}
  std::vector<int> stride;         // e.g. {2,2,2} ; conv_out {1,1,2}
  std::vector<int> padding;        // per-stage
  std::vector<int> dilation;       // {1,1,1}
  std::vector<int> spatial_shape;  // input spatial shape of this stage (x,y,z); 1440->720->360->180
  int kernel_volume{0};            // prod(ksize); filled by ctor if 0
};

class SparseRulebookPrecompute
{
public:
  // out_indices_num_limit must equal the plugin's upper bound (256000) used for the TRT profile
  // max.
  SparseRulebookPrecompute(
    int out_indices_num_limit, std::vector<SparseDownsampleStage> stages, cudaStream_t stream);

  // Compute all stage rulebooks from input voxel coordinates.
  //   coors_d      : device buffer of voxel coords, row-major [num_in, coors_cols].
  //   num_in       : number of input voxels.
  //   coors_cols   : 3 (graph-input [z,y,x], no batch) or 4 ([batch, x, y, z]).
  //   flip_zyx_to_xyz : if true, treat coors as [.. z,y,x] and reverse the spatial columns to
  //                     [x,y,z] before prepending batch (the legacy Autoware graph-input contract).
  // After this returns, stageCount(i) and the device pointers below are valid for binding.
  void compute(const std::int32_t * coors_d, int num_in, int coors_cols, bool flip_zyx_to_xyz);

  int numStages() const { return static_cast<int>(stages_.size()); }
  const SparseDownsampleStage & stage(int i) const { return stages_[i]; }
  int stageCount(int i) const { return stage_counts_[i]; }  // active out-voxels of stage i

  // Device pointers to the (stable, upper-bound-sized) rulebook buffers, valid after compute().
  std::int32_t * outIndices(int i) const { return out_indices_d_[i].get(); }    // [count, 4]
  std::int32_t * pairFwd(int i) const { return pair_fwd_d_[i].get(); }          // [KV, count]
  std::int32_t * pairMask(int i) const { return pair_mask_d_[i].get(); }        // [count, 1]
  std::int32_t * maskArgsort(int i) const { return mask_argsort_d_[i].get(); }  // [count]

private:
  // ---- allocation ---------------------------------------------------------------------------
  void allocateStageBuffers();      // dispatches on reference_mode_
  void allocateReferenceBuffers();  // single shared spconv workspace (reference path)
  void allocateDeviceCountBuffers();  // per-stage persistent state (device-count path)

  // ---- reference path (env BEVFUSION_RULEBOOK_REFERENCE=1) --------------------------------------
  // Run one down-sample stage's rulebook generation via get_indice_pairs_implicit_gemm.
  // Returns num_act_out (host int; spconv performs a D2H inside the call). Cascaded -> 4 D2H syncs.
  int computeStage(int i, const std::int32_t * coords_in_d, int num_in);
  void computeReference(const std::int32_t * coords_in_d, int num_in);

  // ---- device-count path (default) -------------------------------------------------------------
  // Phase 1 (no host sync): stage-1 (hash + per-tap keys) + unique/assign (count -> device,
  // out_indices). ``num_in`` is the stage's input count (== last_num_in_ for stage 0, else the
  // static upper bound N). Writes counts_d_[i] and out_indices_d_[i][0:count].
  void runStage1AndArange(int i, const std::int32_t * coords_in_d, int num_in);
  // Phase 2 (host counts known): stage-2 (tight pair_fwd at num_out) + mask argsort.
  void runStage2AndSort(int i, const std::int32_t * coords_in_d, int num_in, int num_out);
  void computeDeviceCount(int num_in);

  int out_indices_num_limit_;
  std::vector<SparseDownsampleStage> stages_;
  cudaStream_t stream_;
  // When true, use the legacy 4-sync reference path. Set from env at construction for A/B testing.
  bool reference_mode_{false};

  std::vector<int> stage_counts_;
  // Stage-0 input voxel count of the current frame (device-count path; stages>0 use the N bound).
  int last_num_in_{0};

  // Per-stage output (engine-input) buffers, sized to out_indices_num_limit_ (both paths).
  std::vector<CudaUniquePtr<std::int32_t[]>> out_indices_d_;   // [N,4]
  std::vector<CudaUniquePtr<std::int32_t[]>> pair_fwd_d_;      // [KV,N]
  std::vector<CudaUniquePtr<std::int32_t[]>> pair_mask_d_;     // [N,1] (mask_count==1)
  std::vector<CudaUniquePtr<std::int32_t[]>> mask_argsort_d_;  // [N]

  // Scratch: [batch,x,y,z] int32 coords fed to spconv (max num voxels).
  CudaUniquePtr<std::int32_t[]> coords_xyzb_d_{nullptr};

  // ---- reference-path buffers (allocated only when reference_mode_) -----------------------------
  // spconv index-generation workspace + indices_kernel_num + thrust temp (per the plugin layout).
  CudaUniquePtr<std::uint8_t[]> spconv_workspace_d_{nullptr};
  std::size_t spconv_workspace_size_{0};
  // Frame-invariant worst-case theoretical max active-output (max over stages at the N upper bound).
  int max_act_out_theory_worst_{0};

  // ---- device-count-path buffers (allocated only when !reference_mode_) -------------------------
  // These survive Phase 1 -> Phase 2 (per stage), so each is sized for its stage (no reuse).
  std::vector<CudaUniquePtr<std::int32_t[]>> hash_kv_d_;     // [2, hash_size_i] int32 (k row, v row)
  std::vector<CudaUniquePtr<std::int32_t[]>> bkp_d_;         // [KV_i * N + 1] int32 (per-tap out keys)
  std::vector<CudaUniquePtr<std::int32_t[]>> indice_num_d_;  // [KV_i] int32 (stage-1 scratch)
  std::vector<int> hash_size_;                               // hash table capacity per stage
  CudaUniquePtr<std::int32_t[]> counts_d_{nullptr};          // [numStages] active counts (device)
  std::vector<std::int32_t> counts_host_;                    // [numStages] (the single D2H target)
  CudaUniquePtr<std::uint8_t[]> thrust_tmp_d_{nullptr};      // pre-alloc thrust temp for the sort
  CudaUniquePtr<std::int32_t[]> uniq_dummy_d_{nullptr};      // [KV_max*N+1] unused stage-2 arg
};

}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__PREPROCESS__SPARSE_RULEBOOK_PRECOMPUTE_HPP_
