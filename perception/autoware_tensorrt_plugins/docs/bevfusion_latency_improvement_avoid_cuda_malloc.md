# BEVFusion Latency Improvement: Eliminating Hidden `cudaMalloc` in TensorRT Plugins

**Commit:** `a2f50771b7d0c00bf491938e63913b1d5cfdb054`
**PR:** [#12378](https://github.com/autowarefoundation/autoware.universe/pull/12378)
**Title:** `fix(autoware_tensorrt_plugins): avoid tv::zeros and tv::empty`
**Author:** Ryuta Kambe (TIER IV)

---

## Summary

This fix eliminates hidden `cudaMalloc` / `cudaFree` calls that were silently fired on
**every single BEVFusion inference** inside the two sparse-convolution TensorRT plugins:

- `GetIndicesPairsImplicitGemmPlugin` — computes sparse index pairs
- `ImplicitGemmPlugin` — runs the actual sparse convolution (implicit GEMM)

Removing these allocations reduced end-to-end BEVFusion inference latency by **~9 ms**.

---

## Background: Why `cudaMalloc` Is Expensive at Inference Time

`cudaMalloc` is a **blocking, synchronizing** CUDA API call. It:

1. Acquires a global lock on the CUDA context.
2. Forces CPU–GPU synchronization to settle any in-flight work before the allocator
   can safely carve memory.
3. May invoke the OS kernel memory allocator (`mmap`, page tables, etc.).

For short inference pipelines where GPU kernels finish in a few milliseconds, a single
`cudaMalloc` call can easily cost **1–3 ms** of wall-clock time due to the serialization it
introduces.

BEVFusion uses many stacked sparse-convolution layers. Each layer invokes both plugins,
and before this fix each plugin invoked `tv::zeros` / `tv::empty` (which call `cudaMalloc`
internally). With 4+ allocations per sparse-conv layer and ~5 such layers, the total
hidden overhead was **~9 ms per frame**.

An additional consequence: `cudaMalloc` inside `enqueue()` **breaks CUDA graph capture**.
CUDA graphs record GPU operations into a replayable snapshot, but any dynamic allocation
during capture either fails or produces a graph that is incorrect on replay. This had
prevented BEVFusion from benefiting from CUDA graph acceleration at all.

---

## What Was Allocated Per Inference (Before the Fix)

### In `GetIndicesPairsImplicitGemmPlugin::enqueue()`

| Expression                                    | Size                      | Reason for allocation                      |
| --------------------------------------------- | ------------------------- | ------------------------------------------ |
| `tv::zeros({kernel_volume}, tv::int32, 0)`    | `kernel_volume × 4 B`     | Per-kernel activation count, needed zeroed |
| `tv::empty({kernel_volume, N}, tv::int32, 0)` | `kernel_volume × N × 4 B` | Backward index pairs (non-subm only)       |
| `tv::empty({mask_count, N}, tv::int32, 0)`    | `mask_count × N × 4 B`    | Backward pair mask (non-subm only)         |
| `tv::empty({mask_count, N}, tv::int32, 0)`    | `mask_count × N × 4 B`    | Backward mask argsort (non-subm only)      |

`N = out_indices_num_limit_ = 256 000`, `kernel_volume = 27` for a 3×3×3 kernel.
Non-subm allocations alone were **~100 MB** of `cudaMalloc`+`cudaFree` per call.

Thrust (used internally by spconv for radix sort) also issued its own temporary buffer
allocation through CUDA's device memory allocator.

### In `ImplicitGemmPlugin::enqueue()`

| Expression                       | Size | Reason                                    |
| -------------------------------- | ---- | ----------------------------------------- |
| `tv::zeros({1}, tv::uint32, -1)` | 4 B  | CPU-side mask sentinel (all-ones bitmask) |

Although tiny, this was a CPU heap allocation + initialization on **every enqueue**.

---

## How the Fix Works

### 1. All GPU tensors carved from the TensorRT workspace

TensorRT pre-allocates a single workspace buffer before inference begins. Its size is
declared by `getWorkspaceSize()` (called once at engine build time) and the same buffer
is passed to every `enqueue()` call. Tensors created with `tv::from_blob()` wrap an
existing pointer without any allocation.

The workspace layout after this fix:

```bash
┌─────────────────────────────────────────────────┐  offset 0
│  spconv internal workspace                      │
│  (hash tables, temp indices, etc.)              │
│  size: spconv_ws_size                           │
├─────────────────────────────────────────────────┤
│  indices_kernel_num       [KV × 4 B]            │  int32
├─────────────────────────────────────────────────┤
│  (non-subm only)                                │
│  pair_bwd_padded          [KV × N × 4 B]        │  int32
│  pair_mask_bwd_padded     [MC × N × 4 B]        │  int32
│  mask_argsort_bwd_padded  [MC × N × 4 B]        │  int32
├─────────────────────────────────────────────────┤
│  thrust_tmp               [8 MiB]               │  uint8
└─────────────────────────────────────────────────┘
```

`KV` = kernel volume, `N` = 256 000, `MC` = mask count (1 or 2).

The thrust temporary buffer (`kThrustTempBytes = 8 MiB`) gives spconv's internal radix
sort a pre-allocated scratch area so it never falls back to its own `cudaMalloc`.

Zeroing `indices_kernel_num` is now done with `cudaMemsetAsync`, which is non-blocking
and CUDA-graph-compatible.

### 2. `indices_kernel_num` workspace sizing fixed

Before the fix, `getWorkspaceSize()` only reported the spconv-internal workspace size.
This meant the workspace was too small to hold `indices_kernel_num`, and `tv::zeros` was
used as a workaround. The fix adds the sizes of all additional tensors to the reported
workspace size so TensorRT reserves exactly the right amount.

### 3. `mask_tensor_` hoisted to a member variable

In `ImplicitGemmPlugin`, the all-ones mask sentinel:

```cpp
// Before: allocated on every enqueue()
tv::Tensor mask_tensor = tv::zeros({1}, tv::uint32, -1);
mask_tensor.data_ptr<uint32_t>()[0] = 0xffffffff;

// After: allocated once in the constructor, reused every call
mask_tensor_ = tv::zeros({1}, tv::uint32, -1);
mask_tensor_.data_ptr<uint32_t>()[0] = 0xffffffff;
```

This eliminates one CPU heap allocation per inference and is safe because the value
never changes.

---

## Performance Impact

| Source of overhead                          | Before                 | After                           |
| ------------------------------------------- | ---------------------- | ------------------------------- |
| `tv::zeros` / `tv::empty` calls per enqueue | 4 (non-subm)           | 0                               |
| Thrust temp buffer allocation               | per-call (internal)    | Pre-allocated (8 MiB workspace) |
| `mask_tensor` creation                      | per-call               | Once at construction            |
| CUDA graph compatibility                    | Broken (dynamic alloc) | Restored                        |
| **End-to-end BEVFusion latency**            | **baseline**           | **~9 ms faster**                |

The 9 ms saving comes from removing repeated `cudaMalloc` + `cudaFree` round-trips that
each forced CPU–GPU synchronization. With CUDA graph capture now possible, further
savings from graph replay (reduced kernel launch overhead) are also unlocked.

---

## Files Changed

| File                                                     | Change                                                                                                                                                                |
| -------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `src/get_indices_pairs_implicit_gemm_plugin.cpp`         | Replace `tv::zeros` / `tv::empty` with workspace-backed `tv::from_blob`; update `getWorkspaceSize` to cover all tensors; wire `thrust_tmp_tensor_` into the allocator |
| `src/implicit_gemm_plugin.cpp`                           | Remove per-enqueue `tv::zeros` for `mask_tensor`; use pre-allocated member                                                                                            |
| `include/.../get_indices_pairs_implicit_gemm_plugin.hpp` | Add `kThrustTempBytes` constant                                                                                                                                       |
| `include/.../implicit_gemm_plugin.hpp`                   | Add `mask_tensor_` member                                                                                                                                             |
