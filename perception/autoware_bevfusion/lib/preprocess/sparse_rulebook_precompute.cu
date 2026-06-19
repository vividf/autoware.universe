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

#include "autoware/bevfusion/preprocess/sparse_rulebook_precompute.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

// spconv (same headers the GetIndicePairsImplicitGemm plugin uses). cSpell:ignore spconvlib
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/alloc/ThrustAllocator.h>
// ops3d SparseConvIndicesKernel.h exposes the device kernels (calc_conv_indices_*, clean_indices_uniq,
// arange_hash_table_and_assign_out) and the inline ConvProblem / ConvOutLocIter helpers as a
// header-only template surface. The prebuilt libspconv.so exports only the high-level public API
// (get_indice_pairs_implicit_gemm, get_indice_gen_*, get_conv_output_size,
// get_handcrafted_max_act_out) -- the lower-level SpconvOps::generate_conv_inds_* /
// sort_1d_by_key_* / unique_* wrappers are NOT exported. The device-count path below therefore
// launches the underlying header-only __global__ templates directly (replicating spconv's
// stage-1 / unique+assign / stage-2 / sort), without the trailing host count copy and without
// referencing any non-exported symbol. This couples us to spconv's internal kernel layout (vendored
// + pinned); a spconv bump that changes these templates must fail to compile here.
#include <spconvlib/spconv/csrc/sparse/all/ops3d/SparseConvIndicesKernel.h>
#include <tensorview/hash/ops.h>  // tv::hash::clear_map_split (header-only)

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/system/cuda/execution_policy.h>

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace autoware::bevfusion
{

namespace
{
using SpconvOps = spconvlib::spconv::csrc::sparse::all::SpconvOps;
using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
using ThrustAllocator = spconvlib::spconv::csrc::sparse::alloc::ThrustAllocator;

// Header-only spconv device kernels + inline geometry helpers used by the device-count path.
namespace sp3 = spconvlib::spconv::csrc::sparse::all::ops3d;
using ConvProblem3 = spconvlib::spconv::csrc::sparse::all::ops_cpu3d::spinds::ConvProblem;
using ConvLocIter3 = spconvlib::spconv::csrc::sparse::all::ops3d::spinds::ConvOutLocIter;
using ConvLocIter3_64 = spconvlib::spconv::csrc::sparse::all::ops3d::spinds64::ConvOutLocIter;

// kMaskImplicitGemm (matches the exported GetIndicePairsImplicitGemm ``algo`` attribute = 1).
constexpr int kAlgo = 1;
// mask_count: 1 for kMaskImplicitGemm (split-mask would be 2). pair_mask / mask_argsort are [1, N].
constexpr int kMaskCount = 1;
// do_sort: pair-mask argsort. Validated equivalent with sort on; output numerics are independent of
// the argsort ordering (it only schedules the masked gemm). Kept on to match the Python reference.
constexpr bool kDoSort = true;
constexpr std::size_t kThrustTempBytes = 8U * 1024U * 1024U;

// Device-count path: out-of-range coordinate used to pad the unused tail [count, N) of each stage's
// out_indices buffer. The next stage is launched over the static bound N (its real input count is
// only on the device), so these rows must be dropped by spconv's ``query_npq`` bounds check. Any
// value well above the largest stage spatial extent (1440) and far from INT_MAX works.
constexpr std::int32_t kSentinelCoord = 1 << 20;

// coors_d[num_in, cols] -> out[num_in, 4] = [batch=0, x, y, z].
// flip=true : input spatial cols are [z, y, x] (legacy graph-input contract) -> reversed to
// [x,y,z]. cols == 4 : assume input already [batch, x, y, z] and copy verbatim.
__global__ void buildBatchedCoordsKernel(
  const std::int32_t * __restrict__ in, std::int32_t * __restrict__ out, int num_in, int cols,
  bool flip)
{
  const int r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= num_in) {
    return;
  }
  const std::int32_t * src = in + static_cast<std::int64_t>(r) * cols;
  std::int32_t * dst = out + static_cast<std::int64_t>(r) * 4;
  if (cols == 4) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    return;
  }
  // cols == 3
  dst[0] = 0;  // batch
  if (flip) {
    dst[1] = src[2];  // x
    dst[2] = src[1];  // y
    dst[3] = src[0];  // z
  } else {
    dst[1] = src[0];
    dst[2] = src[1];
    dst[3] = src[2];
  }
}

// Device-count path: overwrite out_indices rows [count, n_total) with kSentinelCoord so the next
// down-sample stage (launched over n_total == N) treats them as out-of-range and produces no output.
__global__ void sentinelPadKernel(
  std::int32_t * __restrict__ out_inds, const std::int32_t * __restrict__ count_d, int n_total,
  std::int32_t sentinel)
{
  const int r = blockIdx.x * blockDim.x + threadIdx.x;
  if (r >= n_total) {
    return;
  }
  if (r < *count_d) {
    return;
  }
  std::int32_t * dst = out_inds + static_cast<std::int64_t>(r) * 4;
  dst[0] = 0;  // batch
  dst[1] = sentinel;
  dst[2] = sentinel;
  dst[3] = sentinel;
}

// arange [0, n) used to seed the mask argsort value buffer before the sort_by_key.
__global__ void arangeKernel(std::int32_t * __restrict__ p, int n)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    p[i] = i;
  }
}

bool useInt64HashK(
  const std::vector<int> & spatial, const std::vector<int> & ksize, const std::vector<int> & stride,
  const std::vector<int> & padding, const std::vector<int> & dilation)
{
  auto out_dims = SpconvOps::get_conv_output_size(spatial, ksize, stride, padding, dilation);
  std::int64_t vol = std::accumulate(
    out_dims.begin(), out_dims.end(), static_cast<std::int64_t>(1),
    std::multiplies<std::int64_t>());
  return vol >= static_cast<std::int64_t>(std::numeric_limits<int>::max());
}

void toArray3(const std::vector<int> & v, tv::array<int, 3> & a)
{
  for (int i = 0; i < 3; ++i) {
    a[i] = v[i];
  }
}

// Replicated spconv ``unique_and_assign_output_direct_hash`` (ops3d) WITHOUT the trailing
// ``uniq_cnt.cpu()`` host copy, so the active-output count stays on the device. Iterates the
// (stage-1-populated) hash, assigns each unique output coordinate an arange index (clamped to
// ``num_out_bound``), writes the decoded coordinate to ``out_inds[index]`` and the index back into
// the hash value (so stage-2 can look it up), and accumulates the count into ``uniq_cnt`` (device).
void arangeHashAndAssignOutDeviceCount(
  tv::Tensor hashdata_k, tv::Tensor hashdata_v, tv::Tensor uniq_cnt, tv::Tensor out_inds,
  int num_out_bound, int batch_size, const std::vector<int> & out_dims,
  const std::vector<int> & in_dims, const std::vector<int> & ksize, const std::vector<int> & stride,
  const std::vector<int> & padding, const std::vector<int> & dilation, cudaStream_t stream)
{
  tv::array<int, 3> out_d, in_d, ks, st, pad, dil;
  toArray3(out_dims, out_d);
  toArray3(in_dims, in_d);
  toArray3(ksize, ks);
  toArray3(stride, st);
  toArray3(padding, pad);
  toArray3(dilation, dil);

  tv::cuda::Launch launcher(hashdata_k.size(), stream);
  ConvProblem3 problem(batch_size, 1, 1, in_d, out_d, ks, pad, st, dil);
  const bool use_int32 = problem.check_npq_not_overflow();
  if (num_out_bound <= 0) {
    num_out_bound = static_cast<int>(hashdata_k.size());
  }
  if (static_cast<int>(use_int32) == 0) {
    ConvLocIter3_64 loc_iter(problem);
    tv::dispatch<int32_t, int64_t>(hashdata_k.dtype(), [&](auto I) {
      using V = int32_t;
      using K = TV_DECLTYPE(I);
      using table_t = tv::hash::LinearHashTableSplit<
        K, V, tv::hash::Murmur3Hash<K>, tv::hash::default_empty_key_v<K>, false>;
      table_t table(hashdata_k.data_ptr<K>(), hashdata_v.data_ptr<V>(), hashdata_k.dim(0));
      launcher(
        sp3::arange_hash_table_and_assign_out<table_t, std::decay_t<decltype(loc_iter.layout_npq)>>,
        table, out_inds.data_ptr<int>(), uniq_cnt.data_ptr<int>(), num_out_bound,
        loc_iter.layout_npq);
    });
  } else {
    ConvLocIter3 loc_iter(problem);
    tv::dispatch<int32_t, int64_t>(hashdata_k.dtype(), [&](auto I) {
      using V = int32_t;
      using K = TV_DECLTYPE(I);
      using table_t = tv::hash::LinearHashTableSplit<
        K, V, tv::hash::Murmur3Hash<K>, tv::hash::default_empty_key_v<K>, false>;
      table_t table(hashdata_k.data_ptr<K>(), hashdata_v.data_ptr<V>(), hashdata_k.dim(0));
      launcher(
        sp3::arange_hash_table_and_assign_out<table_t, std::decay_t<decltype(loc_iter.layout_npq)>>,
        table, out_inds.data_ptr<int>(), uniq_cnt.data_ptr<int>(), num_out_bound,
        loc_iter.layout_npq);
    });
  }
  // NOTE: spconv's unique_and_assign_output_direct_hash ends here with ``uniq_cnt.cpu()`` (a blocking
  // D2H). We deliberately omit it: the count stays on the device and is fetched for all stages at
  // once by a single D2H in computeDeviceCount(). This is the whole point of the device-count path.
}
}  // namespace

SparseRulebookPrecompute::SparseRulebookPrecompute(
  int out_indices_num_limit, std::vector<SparseDownsampleStage> stages, cudaStream_t stream)
: out_indices_num_limit_(out_indices_num_limit), stages_(std::move(stages)), stream_(stream)
{
  for (auto & s : stages_) {
    if (s.kernel_volume == 0) {
      s.kernel_volume = std::accumulate(s.ksize.begin(), s.ksize.end(), 1, std::multiplies<int>());
    }
  }
  stage_counts_.assign(stages_.size(), 0);

  // A/B verification toggle: BEVFUSION_RULEBOOK_REFERENCE=1 selects the legacy 4-sync path so the
  // device-count rulebooks can be compared against it on identical inputs.
  const char * ref_env = std::getenv("BEVFUSION_RULEBOOK_REFERENCE");
  reference_mode_ = (ref_env != nullptr && ref_env[0] == '1');

  allocateStageBuffers();
}

void SparseRulebookPrecompute::allocateStageBuffers()
{
  if (reference_mode_) {
    allocateReferenceBuffers();
  } else {
    allocateDeviceCountBuffers();
  }
}

void SparseRulebookPrecompute::allocateReferenceBuffers()
{
  const int N = out_indices_num_limit_;

  int max_kv = 0;
  bool any_int64_hash = false;
  // Worst-case max_act_out_in_theory over all stages at the maximum input count (N). Each runtime
  // stage uses num_in <= N, so this bounds the largest workspace any stage can carve. Stored as a
  // member and reused by computeStage() every frame so the workspace layout/offsets are frame-
  // invariant (no per-frame get_handcrafted_max_act_out()).
  max_act_out_theory_worst_ = 0;
  for (const auto & s : stages_) {
    max_act_out_theory_worst_ = std::max(
      max_act_out_theory_worst_, SpconvOps::get_handcrafted_max_act_out(
                                   static_cast<std::size_t>(N), s.ksize, s.stride, s.padding,
                                   s.dilation));
  }
  const int max_act_out_theory = max_act_out_theory_worst_;
  for (const auto & s : stages_) {
    max_kv = std::max(max_kv, s.kernel_volume);
    any_int64_hash =
      any_int64_hash || useInt64HashK(s.spatial_shape, s.ksize, s.stride, s.padding, s.dilation);

    out_indices_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4));
    pair_fwd_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(
        static_cast<std::size_t>(s.kernel_volume) * N));
    pair_mask_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
    mask_argsort_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
  }

  coords_xyzb_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4);

  // Workspace layout mirrors get_indices_pairs_implicit_gemm_plugin.cpp (non-subm):
  //   [ spconv index-gen workspace ]
  //   [ indices_kernel_num : KV int32 ]
  //   [ pair_bwd           : KV * N int32 ]
  //   [ pair_mask_bwd      : mask_count * N int32 ]
  //   [ mask_argsort_bwd   : mask_count * N int32 ]
  //   [ thrust_tmp         : kThrustTempBytes ]
  // Sized for the largest stage (max kernel_volume) and reused across stages.
  const bool use_direct_table = true;  // non-subm
  std::size_t spconv_ws = static_cast<std::size_t>(SpconvOps::get_indice_gen_workspace_size(
    max_kv, N, N, max_act_out_theory, /*is_subm=*/false, any_int64_hash, use_direct_table));
  std::size_t sz = spconv_ws;
  sz += static_cast<std::size_t>(max_kv) * sizeof(std::int32_t);          // indices_kernel_num
  sz += static_cast<std::size_t>(max_kv) * N * sizeof(std::int32_t);      // pair_bwd
  sz += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);  // pair_mask_bwd
  sz += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);  // mask_argsort_bwd
  sz += kThrustTempBytes;

  spconv_workspace_size_ = sz;
  spconv_workspace_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(sz);
}

void SparseRulebookPrecompute::allocateDeviceCountBuffers()
{
  const int N = out_indices_num_limit_;
  const int num_stages = numStages();

  hash_size_.assign(num_stages, 0);
  int max_kv = 0;
  for (int i = 0; i < num_stages; ++i) {
    const auto & s = stages_[i];
    // The replicated unique+assign uses an int32 hash/key buffer. int64 hashing is only needed when
    // the output spatial volume exceeds INT_MAX (not the case for the BEVFusion-L down-sample
    // stages). Refuse it loudly rather than silently mis-hashing.
    if (useInt64HashK(s.spatial_shape, s.ksize, s.stride, s.padding, s.dilation)) {
      throw std::runtime_error(
        "SparseRulebookPrecompute device-count path: stage '" + s.onnx_base +
        "' needs an int64 hash (unsupported here). Set BEVFUSION_RULEBOOK_REFERENCE=1 or extend the "
        "device-count buffers to int64.");
    }
    max_kv = std::max(max_kv, s.kernel_volume);

    // hash table capacity == high-level ``int(max_act_out_in_theory * 1.1)`` evaluated at the N
    // upper bound (>= any per-frame need since num_in <= N), so the buffers are frame-invariant.
    const int max_act = SpconvOps::get_handcrafted_max_act_out(
      static_cast<std::size_t>(N), s.ksize, s.stride, s.padding, s.dilation);
    hash_size_[i] = static_cast<int>(max_act * 1.1);

    // Persistent per-stage state carried from Phase 1 (stage-1 + unique) to Phase 2 (stage-2 +
    // sort): the direct-table hash and the per-tap output-coordinate keys.
    hash_kv_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(
        static_cast<std::size_t>(2) * hash_size_[i]));
    bkp_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(
        static_cast<std::size_t>(s.kernel_volume) * N + 1));
    indice_num_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(s.kernel_volume)));

    // Engine-input (rulebook) buffers, upper-bound sized.
    out_indices_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4));
    pair_fwd_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(
        static_cast<std::size_t>(s.kernel_volume) * N));
    pair_mask_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
    mask_argsort_d_.emplace_back(
      autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(kMaskCount) * N));
  }

  coords_xyzb_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(N) * 4);
  counts_d_ =
    autoware::cuda_utils::make_unique<std::int32_t[]>(static_cast<std::size_t>(num_stages));
  counts_host_.assign(num_stages, 0);
  thrust_tmp_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(kThrustTempBytes);
  // Unused-but-required ``indice_pairs_uniq`` arg of the high-level stage-2 wrapper is not needed
  // here (we launch the inference kernel directly), but keep a scratch around for safety/symmetry.
  uniq_dummy_d_ = autoware::cuda_utils::make_unique<std::int32_t[]>(
    static_cast<std::size_t>(max_kv) * N + 1);
}

int SparseRulebookPrecompute::computeStage(int i, const std::int32_t * coords_in_d, int num_in)
{
  const SparseDownsampleStage & s = stages_[i];
  const int N = out_indices_num_limit_;
  const int kv = s.kernel_volume;

  std::vector<int> ksize = s.ksize, stride = s.stride, padding = s.padding, dilation = s.dilation;
  std::vector<int> input_dims = s.spatial_shape;
  const bool use_direct_table = true;
  const bool use_int64_hash_k = useInt64HashK(s.spatial_shape, ksize, stride, padding, dilation);

  // max_act_out_in_theory sizes the internal indice_pairs_uniq buffer (~max_act_out_in_theory * 1.1).
  // Using N here (as the earliest code did) under-allocates for the large down-sample stages and trips
  // the StaticAllocator "tensor size too small" assert. The plugin derives it from the per-call input
  // count; we instead reuse the frame-invariant worst case (max over stages at the N upper bound,
  // computed once in allocateReferenceBuffers). Since num_in <= N and get_handcrafted_max_act_out is
  // non-decreasing in the input count, this always bounds the per-frame need, while keeping the
  // workspace carving/offsets identical every frame (no per-frame get_handcrafted_max_act_out()).
  const int max_act_out_theory = max_act_out_theory_worst_;

  // Carve the workspace exactly like the plugin.
  std::uint8_t * ws = spconv_workspace_d_.get();
  std::size_t spconv_ws_size = static_cast<std::size_t>(SpconvOps::get_indice_gen_workspace_size(
    kv, N, N, max_act_out_theory, /*is_subm=*/false, use_int64_hash_k, use_direct_table));

  auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
    ws, kv, N, N, max_act_out_theory, /*is_subm=*/false, use_int64_hash_k, use_direct_table);

  std::uint8_t * indice_num_ptr = ws + spconv_ws_size;
  tv::Tensor indices_kernel_num = tv::from_blob(indice_num_ptr, {kv}, tv::int32, 0);
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    indice_num_ptr, 0, static_cast<std::size_t>(kv) * sizeof(std::int32_t), stream_));

  std::uint8_t * extra = indice_num_ptr + static_cast<std::size_t>(kv) * sizeof(std::int32_t);
  tv::Tensor pair_bwd = tv::from_blob(extra, {kv, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kv) * N * sizeof(std::int32_t);
  tv::Tensor pair_mask_bwd = tv::from_blob(extra, {kMaskCount, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);
  tv::Tensor mask_argsort_bwd = tv::from_blob(extra, {kMaskCount, N}, tv::int32, 0);
  extra += static_cast<std::size_t>(kMaskCount) * N * sizeof(std::int32_t);
  tv::Tensor thrust_tmp =
    tv::from_blob(extra, {static_cast<std::int64_t>(kThrustTempBytes)}, tv::uint8, 0);

  // Outputs go to our stable per-stage buffers (= the engine input tensors).
  tv::Tensor out_indices = tv::from_blob(out_indices_d_[i].get(), {N, 4}, tv::int32, 0);
  tv::Tensor pair_fwd = tv::from_blob(pair_fwd_d_[i].get(), {kv, N}, tv::int32, 0);
  tv::Tensor pair_mask_fwd = tv::from_blob(pair_mask_d_[i].get(), {kMaskCount, N}, tv::int32, 0);
  tv::Tensor mask_argsort_fwd =
    tv::from_blob(mask_argsort_d_[i].get(), {kMaskCount, N}, tv::int32, 0);

  tv::Tensor input_indices =
    tv::from_blob(const_cast<std::int32_t *>(coords_in_d), {num_in, 4}, tv::int32, 0);

  ws_tensors.emplace(SPCONV_ALLOC_PAIR_FWD, pair_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_BWD, pair_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_MASK, pair_mask_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_PAIR_MASK_BWD, pair_mask_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_MASK_ARG_SORT, mask_argsort_fwd);
  ws_tensors.emplace(SPCONV_ALLOC_MASK_ARG_SORT_BWD, mask_argsort_bwd);
  ws_tensors.emplace(SPCONV_ALLOC_OUT_INDICES, out_indices);
  ws_tensors.emplace(SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num);  // cSpell:ignore INDICE

  StaticAllocator alloc(ws_tensors);
  alloc.thrust_tmp_tensor_ = thrust_tmp;

  // cSpell:ignore indice
  auto pair_res = SpconvOps::get_indice_pairs_implicit_gemm(
    alloc, input_indices, /*batch_size=*/1, input_dims, kAlgo, ksize, stride, padding, dilation,
    {0, 0, 0}, /*subm=*/false, /*transpose=*/false, /*is_train=*/false,
    reinterpret_cast<std::uintptr_t>(stream_), N, tv::CUDAKernelTimer(false), use_direct_table,
    kDoSort);

  return std::get<1>(pair_res);  // num_act_out (host int; spconv performs the single D2H here)
}

void SparseRulebookPrecompute::computeReference(const std::int32_t * coords_in_d, int num_in)
{
  // Cascade: each down-sample's out_indices feed the next (submanifold layers in between do not
  // change coordinates, so they are not part of this cascade — they stay in the TRT graph). Each
  // computeStage() does an internal D2H of its count -> 4 blocking syncs total.
  const std::int32_t * cur_coords = coords_in_d;
  int cur_num = num_in;
  for (int i = 0; i < numStages(); ++i) {
    int n_out = computeStage(i, cur_coords, cur_num);
    stage_counts_[i] = n_out;
    cur_coords = out_indices_d_[i].get();  // [n_out, 4]
    cur_num = n_out;
  }
}

void SparseRulebookPrecompute::runStage1AndArange(
  int i, const std::int32_t * coords_in_d, int num_in)
{
  const SparseDownsampleStage & s = stages_[i];
  const int N = out_indices_num_limit_;
  const int kv = s.kernel_volume;
  const auto out_shape =
    SpconvOps::get_conv_output_size(s.spatial_shape, s.ksize, s.stride, s.padding, s.dilation);

  tv::array<int, 3> out_d, in_d, ks, st, pad, dil;
  toArray3(out_shape, out_d);
  toArray3(s.spatial_shape, in_d);
  toArray3(s.ksize, ks);
  toArray3(s.stride, st);
  toArray3(s.padding, pad);
  toArray3(s.dilation, dil);

  auto hash_kv = tv::from_blob(hash_kv_d_[i].get(), {2, hash_size_[i]}, tv::int32, 0);
  tv::Tensor hash_k = hash_kv[0];
  tv::Tensor hash_v = hash_kv[1];
  const std::int64_t uniq_size = static_cast<std::int64_t>(kv) * num_in + 1;
  tv::Tensor bkp = tv::from_blob(bkp_d_[i].get(), {uniq_size}, tv::int32, 0);

  // stage-1 internally clears the hash (clear_map_split) and the per-tap key buffer
  // (clean_indices_uniq) below, so we only zero indice_num. counts_d_[i] is zeroed once for all
  // stages in computeDeviceCount before Phase 1.
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    indice_num_d_[i].get(), 0, static_cast<std::size_t>(kv) * sizeof(std::int32_t), stream_));

  ConvProblem3 problem(1, 1, 1, in_d, out_d, ks, pad, st, dil);
  const bool use_int32 = problem.check_npq_not_overflow();

  // ---- stage-1: clear hash, clean per-tap keys, build conv indices (mask, direct table). Replicates
  // ops3d::SparseConvIndicesKernel::generate_conv_inds_mask_stage1_direct_table (not exported). ----
  tv::dispatch<int32_t, int64_t>(bkp.dtype(), [&](auto I) {
    using V = int32_t;
    using T = TV_DECLTYPE(I);
    using K = TV_DECLTYPE(I);
    using table_t = tv::hash::LinearHashTableSplit<
      K, V, tv::hash::Murmur3Hash<K>, tv::hash::default_empty_key_v<K>, false>;
    table_t table(hash_k.data_ptr<K>(), hash_v.data_ptr<V>(), hash_k.dim(0));
    tv::hash::clear_map_split(table, stream_);

    tv::cuda::Launch clean_launcher(static_cast<std::size_t>(uniq_size), stream_);
    clean_launcher(
      sp3::clean_indices_uniq<T>, bkp.data_ptr<T>(), static_cast<std::size_t>(uniq_size));

    tv::cuda::Launch launcher(static_cast<std::size_t>(num_in), stream_);
    launcher.blocks.y = kv;
    // indice_pairs_bwd is unused on the non-subm direct-table path (the kernel's write is disabled),
    // so pass nullptr; pair-bwd is only produced for training.
    if (static_cast<int>(use_int32) == 0) {
      ConvLocIter3_64 loc(problem);
      launcher(
        sp3::calc_conv_indices_stage1_mask_direct_table<T, table_t, ConvLocIter3_64>, table, loc,
        coords_in_d, static_cast<std::int32_t *>(nullptr), bkp.data_ptr<T>(),
        indice_num_d_[i].get(), num_in, kv, /*transposed=*/false);
    } else {
      ConvLocIter3 loc(problem);
      launcher(
        sp3::calc_conv_indices_stage1_mask_direct_table<T, table_t, ConvLocIter3>, table, loc,
        coords_in_d, static_cast<std::int32_t *>(nullptr), bkp.data_ptr<T>(),
        indice_num_d_[i].get(), num_in, kv, /*transposed=*/false);
    }
  });

  // ---- unique + assign (count -> device): no host sync. ----
  tv::Tensor out_inds = tv::from_blob(out_indices_d_[i].get(), {N, 4}, tv::int32, 0);
  tv::Tensor uniq_cnt = tv::from_blob(counts_d_.get() + i, {1}, tv::int32, 0);
  arangeHashAndAssignOutDeviceCount(
    hash_k, hash_v, uniq_cnt, out_inds, /*num_out_bound=*/N, /*batch_size=*/1, out_shape,
    s.spatial_shape, s.ksize, s.stride, s.padding, s.dilation, stream_);
}

void SparseRulebookPrecompute::runStage2AndSort(
  int i, const std::int32_t * coords_in_d, int num_in, int num_out)
{
  const SparseDownsampleStage & s = stages_[i];
  const int kv = s.kernel_volume;
  const int mask_int_count = (kv + 31) / 32;  // div_up(kv, 32) == 1 for kv <= 32
  (void)coords_in_d;  // stage-2 inference path reads the per-tap keys + hash, not the input coords

  auto hash_kv = tv::from_blob(hash_kv_d_[i].get(), {2, hash_size_[i]}, tv::int32, 0);
  tv::Tensor hash_k = hash_kv[0];
  tv::Tensor hash_v = hash_kv[1];
  // bkp must use the SAME num_in (row stride) as stage-1 wrote it with in Phase 1.
  const std::int64_t uniq_size = static_cast<std::int64_t>(kv) * num_in + 1;
  tv::Tensor bkp = tv::from_blob(bkp_d_[i].get(), {uniq_size}, tv::int32, 0);

  // Tight output-sized rulebook buffers: row stride == num_out, matching the engine input shape set
  // by setSparseRulebookInputShapes(). out_indices was already packed [0, num_out) by Phase 1.
  // pair_fwd defaults to -1 (taps with no input); stage-2 atomicOrs mask_fwd so it must start at 0.
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    pair_fwd_d_[i].get(), 0xFF, static_cast<std::size_t>(kv) * num_out * sizeof(std::int32_t),
    stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    pair_mask_d_[i].get(), 0, static_cast<std::size_t>(num_out) * sizeof(std::int32_t), stream_));

  // ---- stage-2 inference mask path: fill pair_fwd + pair_mask via the hash. Replicates
  // ops3d::SparseConvIndicesKernel::generate_conv_inds_stage2_mask_direct_table (not exported),
  // inference branch (mask_bwd empty). ----
  tv::cuda::Launch launcher(static_cast<std::size_t>(num_in), stream_);
  launcher.blocks.y = kv;
  tv::dispatch<int32_t, int64_t>(hash_k.dtype(), [&](auto I) {
    using V = int32_t;
    using K = TV_DECLTYPE(I);
    using table_t = tv::hash::LinearHashTableSplit<
      K, V, tv::hash::Murmur3Hash<K>, tv::hash::default_empty_key_v<K>, false>;
    table_t hash(hash_k.data_ptr<K>(), hash_v.data_ptr<V>(), hash_k.dim(0));
    launcher(
      sp3::calc_conv_indices_stage2_inference_mask<table_t, true>, hash, pair_fwd_d_[i].get(),
      static_cast<std::int32_t *>(nullptr), bkp.data_ptr<K>(),
      reinterpret_cast<std::uint32_t *>(pair_mask_d_[i].get()), num_in, num_out, mask_int_count);
  });

  // ---- mask argsort: arange then thrust sort_by_key (key = mask uint32, value = argsort). Matches
  // SpconvOps::sort_1d_by_key_allocator_v2 (not exported). Thrust temp is served from our preallocated
  // buffer via the StaticAllocator, so the sort issues no cudaMalloc. ----
  const int block = 256;
  const int grid = (num_out + block - 1) / block;
  arangeKernel<<<grid, block, 0, stream_>>>(mask_argsort_d_[i].get(), num_out);
  CHECK_CUDA_ERROR(cudaGetLastError());

  std::unordered_map<std::string, tv::Tensor> empty_dict;
  StaticAllocator salloc(empty_dict);
  salloc.thrust_tmp_tensor_ =
    tv::from_blob(thrust_tmp_d_.get(), {static_cast<std::int64_t>(kThrustTempBytes)}, tv::uint8, 0);
  ThrustAllocator thrustalloc(salloc);
  thrust::device_ptr<std::uint32_t> key_ptr(reinterpret_cast<std::uint32_t *>(pair_mask_d_[i].get()));
  thrust::device_ptr<std::int32_t> val_ptr(mask_argsort_d_[i].get());
  thrust::sort_by_key(
    thrust::cuda::par(thrustalloc).on(stream_), key_ptr, key_ptr + num_out, val_ptr);
}

void SparseRulebookPrecompute::computeDeviceCount(int num_in)
{
  const int num_stages = numStages();
  const int N = out_indices_num_limit_;
  last_num_in_ = num_in;

  // arange uses atomicAggInc into counts_d_, so zero all counts before Phase 1.
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    counts_d_.get(), 0, static_cast<std::size_t>(num_stages) * sizeof(std::int32_t), stream_));

  // ---- Phase 1: GPU-resident cascade. Each stage's active count lands in counts_d_[i]. No host
  // sync, so the whole cascade is enqueued on the stream back-to-back. -------------------------
  const std::int32_t * cur_coords = coords_xyzb_d_.get();
  int cur_num = num_in;  // stage 0 processes the real voxel count
  for (int i = 0; i < num_stages; ++i) {
    runStage1AndArange(i, cur_coords, cur_num);
    if (i + 1 < num_stages) {
      // Pad the unused tail [count_i, N) so the next stage (launched over N, since count_i is not
      // yet on the host) drops it as out-of-range.
      const int block = 256;
      const int grid = (N + block - 1) / block;
      sentinelPadKernel<<<grid, block, 0, stream_>>>(
        out_indices_d_[i].get(), counts_d_.get() + i, N, kSentinelCoord);
      CHECK_CUDA_ERROR(cudaGetLastError());
    }
    cur_coords = out_indices_d_[i].get();
    cur_num = N;  // stages > 0 are processed up to the upper bound (sentinel-padded tail)
  }

  // ---- The single device-to-host copy of all stage counts (replaces the 4 per-stage syncs). ----
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    counts_host_.data(), counts_d_.get(), static_cast<std::size_t>(num_stages) * sizeof(std::int32_t),
    cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // ---- Phase 2: build the tight (output-stride) rulebooks now that the counts are known. --------
  for (int i = 0; i < num_stages; ++i) {
    int n_out = counts_host_[i];
    if (n_out <= 0) {
      throw std::runtime_error(
        "SparseRulebookPrecompute: stage '" + stages_[i].onnx_base + "' produced 0 active voxels");
    }
    if (n_out > N) {
      n_out = N;  // matches the arange clamp to num_out_bound == N
    }
    stage_counts_[i] = n_out;
    const int nin = (i == 0) ? last_num_in_ : N;
    const std::int32_t * in_coords = (i == 0) ? coords_xyzb_d_.get() : out_indices_d_[i - 1].get();
    runStage2AndSort(i, in_coords, nin, n_out);
  }
}

void SparseRulebookPrecompute::compute(
  const std::int32_t * coors_d, int num_in, int coors_cols, bool flip_zyx_to_xyz)
{
  if (num_in <= 0) {
    throw std::runtime_error("SparseRulebookPrecompute: num_in must be > 0");
  }

  // Build [batch, x, y, z] int32 coords for the first down-sample stage.
  const int block = 256;
  const int grid = (num_in + block - 1) / block;
  buildBatchedCoordsKernel<<<grid, block, 0, stream_>>>(
    coors_d, coords_xyzb_d_.get(), num_in, coors_cols, flip_zyx_to_xyz);
  CHECK_CUDA_ERROR(cudaGetLastError());

  if (reference_mode_) {
    computeReference(coords_xyzb_d_.get(), num_in);
  } else {
    computeDeviceCount(num_in);
  }
}

}  // namespace autoware::bevfusion
