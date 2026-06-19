# BEVFusion Sparse-Rulebook Preprocessing — Device-Count Optimization Profiling Report

## TL;DR

The device-count sparse-rulebook precompute (commit `3ecbf51`, goal: **4 device→host
syncs → 1**) **did not improve preprocessing — it slightly regressed it: +3.8 % GPU
time per frame.**

The sync reduction itself worked (~3 fewer `cudaMemcpyAsync` D2H per frame), but to chain
the four down-sample stages on the GPU *without* a per-stage host count, every stage must
be launched at the static upper bound `N = 256000` instead of its real (shrinking) active
count. That over-launch inflated the rulebook-precompute GPU compute by **+72.5 %**, which
outweighs the saved syncs.

| Verdict | Δ preprocess GPU / frame |
|---|---|
| ❌ Net regression | **+3.8 %** (+101 µs) |

---

## Background

The optimized sparse ONNX has its 4 down-sample `GetIndicePairsImplicitGemm` nodes removed
and their rulebooks exposed as graph inputs; we precompute them at runtime
(`SparseRulebookPrecompute`, `autoware_bevfusion/lib/preprocess/sparse_rulebook_precompute.cu`).

- **Before (`no_opt`)** — reference path: one `spconv::get_indice_pairs_implicit_gemm` call
  per stage. Each call does an internal `unique_hash → .cpu()` (a blocking D2H of the active
  count) that it needs *within the same call* to size stage-2. 4 stages ⇒ **4 blocking
  D2H syncs**, serialized by the stage cascade. (Selectable via
  `BEVFUSION_RULEBOOK_REFERENCE=1`.)
- **After (`opt`)** — device-count path: stage-1/unique/stage-2/sort are launched directly
  from the header-only spconv kernels with the active count kept on the device; all 4 counts
  are copied to the host in **one** D2H. To launch each stage without the previous stage's
  host count, inputs are processed up to `N` with an out-of-range sentinel tail.

## Method

Two Nsight Systems captures of the same node + bag (`only_concat`, lidar-only):

- `bevfusion_profile_no_opt_preprocessing.nsys-rep` (before)
- `bevfusion_profile_opt_preprocessing.nsys-rep`  (after)

Compared with [`compare_bevfusion_preprocess.py`](compare_bevfusion_preprocess.py):

- **Per-frame normalized** — the two captures have different frame counts (43 vs 30); all
  numbers are divided by the counted frame count.
- **Warm-up excluded** — the first frame (engine warm-up / graph capture; ~400 ms enqueue)
  is dropped.
- **Region split by timestamp** — a GPU kernel is *inference* iff it starts inside a
  `TensorRT:ExecutionContext::enqueue` NVTX range, else *preprocess*. This is robust to
  spconv kernel renames (no hard-coded kernel list); validated: the in-graph submanifold
  kernels (`calc_subm_*`) land inside enqueue, our down-sample rulebook kernels
  (`calc_conv_indices_stage1/2_*`, `arange_hash_table*`) land outside.

## Results (per frame, warm-up excluded)

```
                                     BEFORE       AFTER       DELTA
frames (total / counted)            43/42       30/29
warm-up max enqueue (ms)            405.3       382.5
---------------------------------------------------------------------
PER-FRAME GPU kernel time (us)
  preprocess (outside enqueue)     2677.18     2778.26    +101.08  (+3.8%)
  inference  (inside enqueue)      2516.57     2485.56     -31.01  (-1.2%)   # control ~equal
    - rulebook precompute           199.53      344.21    +144.68  (+72.5%)  # <-- over-launch cost
    - voxelization                  102.58       99.39      -3.19  (-3.1%)
    - other (sorts, clears, ...)   2375.08     2334.66     -40.42  (-1.7%)
---------------------------------------------------------------------
PER-FRAME host CUDA API (total; inference constant, so Δ ≈ preprocess change)
  cudaMemcpyAsync   count/frame      47.17       44.24      -2.93  (-6.2%)   # <-- 4->1 D2H worked
                    time  us/frame  516.36      473.80     -42.55  (-8.2%)
  cudaMemcpy        count/frame       2.00        2.00       0.00
  cudaStreamSync    count/frame      47.48       48.69      +1.21  (+2.6%)
  cudaLaunchKernel  count/frame     374.31      363.45     -10.86  (-2.9%)
```

## Analysis

- **The optimization did what it intended.** `cudaMemcpyAsync` (D2H) dropped by ~3 calls /
  frame (−8.2 % API time) — the 4→1 sync reduction is real.
- **But the enabling trick costs more than it saves.** Keeping the cascade host-sync-free
  means each stage is launched at `N = 256000` (the sentinel-padded upper bound) instead of
  its real active count, which shrinks per level (≈ voxels → /≈ 2³ per stride-2 stage). The
  reference path processes the real, shrinking counts. Result: rulebook GPU compute
  **199.5 → 344.2 µs/frame (+72.5 %, +145 µs)**.
- **Net:** the saved syncs (~42 µs of API time, mostly overlapped with the GPU and hidden)
  do not offset +145 µs of extra GPU compute on the critical path ⇒ **preprocess GPU total
  +3.8 % (+101 µs/frame)**. Inference is unchanged (−1.2 %, noise), confirming the delta is
  isolated to preprocessing.

This is the "decouple-the-cascade ⇒ forced over-launch" trade-off: removing the host syncs
requires processing the worst-case size every stage, and on sparse scenes that over-launch
exceeds the sync savings.

## Recommendations

1. **2-sync variant (recommended).** After stage 0, sync `count_0` to the host once and use
   it (not `N`) as the input bound for stages 1–3. Stages 1–3 then process the real working
   size — removing most of the over-launch — while still cutting 4 syncs to 2. Expected to be
   neutral-to-slightly-positive. Re-profile with the script to confirm.
2. **Otherwise revert** to the reference path (`BEVFUSION_RULEBOOK_REFERENCE=1`), which is
   the faster of the two measured here.

## Reproduce

```bash
python3 compare_bevfusion_preprocess.py \
    bevfusion_profile_no_opt_preprocessing.nsys-rep \
    bevfusion_profile_opt_preprocessing.nsys-rep [--skip-warmup 1]
```

The script (re)exports each `.nsys-rep` to sqlite, splits preprocess vs inference by NVTX
timestamp, normalizes per frame, and prints the table above with a verdict.
