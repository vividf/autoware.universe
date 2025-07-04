# Test Time Augmentation (TTA) Implementation for CenterPoint

This document describes the Test Time Augmentation (TTA) implementation for the Autoware CenterPoint 3D object detection system.

## Overview

TTA is a technique that applies data augmentation during inference to improve detection performance. This implementation uses rotation augmentations with 10°, 120°, and 240° angles, running them in parallel on GPU to maximize performance.

## Features

### 1. Rotation Augmentations

- **0° (Original)**: No rotation, baseline detection
- **90°**: Quarter rotation for moderate augmentation
- **180°**: Half rotation for significant augmentation

### 2. Parallel Processing

- All augmentations run in parallel on GPU
- Efficient memory usage with shared CUDA streams
- Optimized for high-throughput inference

### 3. Result Merging

- Non-Maximum Suppression (NMS) based merging
- IoU threshold configurable (default: 0.5)
- Transforms detections back to original coordinate system

## Architecture

### Core Components

1. **TTAConfig**: Configuration structure for TTA parameters
2. **TTAProcessor**: Main TTA processing class
3. **TTAResult**: Individual augmentation result structure
4. **CUDA Kernels**: Parallel processing kernels for GPU acceleration

### File Structure

```bash
perception/autoware_lidar_centerpoint/
├── include/autoware/lidar_centerpoint/
│   ├── tta_config.hpp          # TTA configuration
│   ├── tta_processor.hpp       # TTA processor interface
│   └── preprocess/
│       └── tta_kernel.hpp      # CUDA kernel headers
├── lib/
│   ├── tta_processor.cpp       # TTA processor implementation
│   ├── centerpoint_trt.cpp     # Modified with TTA support
│   └── preprocess/
│       └── tta_kernel.cu       # CUDA kernels
├── config/
│   └── tta_config.yaml         # TTA configuration file
└── src/
    └── node.cpp                # Modified node with TTA
```

## Usage

### 1. Configuration

TTA can be configured through ROS parameters:

```yaml
tta:
  enabled: true
  rotation_angles: [0.0, 10.0, 120.0, 240.0]
  iou_threshold: 0.5
  enable_parallel: true
```

### 2. Runtime Usage

The TTA functionality is automatically enabled when the node starts. The system will:

1. Generate augmented point clouds using the specified rotation angles
2. Process each augmentation through the CenterPoint pipeline
3. Merge results using NMS
4. Return the final detection results

### 3. Performance Considerations

- **GPU Memory**: TTA requires approximately 3x more GPU memory (for 3 augmentations)
- **Processing Time**: Parallel processing minimizes time overhead
- **Accuracy**: Typically improves detection accuracy, especially for rotated objects

## Implementation Details

### 1. Point Cloud Augmentation

```cpp
// Generate rotation transformation
Eigen::Matrix4f transform = generateRotationTransform(angle_degrees);

// Apply transformation to point cloud
applyTransform(input_points, num_points, transform, output_points);
```

### 2. Parallel GPU Processing

```cpp
// CUDA kernel for parallel rotation
rotatePointsParallelKernel<<<grid_dim, block_dim, 0, stream>>>(
  input_points, num_points, point_feature_size,
  rotation_angles, num_augmentations, output_points);
```

### 3. Result Merging

```cpp
// Transform detections back to original coordinate system
auto transformed_boxes = transformBoxes(detected_boxes, inverse_transform);

// Merge using NMS
auto merged_boxes = nmsMerge(all_boxes, iou_threshold);
```

## Performance Metrics

### Expected Improvements

- **Detection Accuracy**: 5-15% improvement in mAP
- **Processing Time**: ~2-3x increase (parallel processing helps)
- **Memory Usage**: ~3x increase for 3 augmentations

### Memory Requirements

For a typical configuration:

- **Original**: ~2GB GPU memory
- **With TTA**: ~6GB GPU memory (3 augmentations)

## Troubleshooting

### Common Issues

1. **Out of Memory**: Reduce number of augmentations or batch size
2. **Poor Performance**: Check GPU utilization and memory bandwidth
3. **Incorrect Results**: Verify transformation matrices and coordinate systems

### Debugging

Enable debug output by setting the logger level:

```bash
ros2 run autoware_lidar_centerpoint autoware_lidar_centerpoint_node --ros-args --log-level debug
```

## Future Enhancements

1. **Additional Augmentations**: Scale, translation, noise
2. **Adaptive TTA**: Dynamic augmentation based on scene complexity
3. **Ensemble Methods**: Weighted averaging of augmentation results
4. **Memory Optimization**: Streaming augmentations to reduce memory usage

## References

- [CenterPoint Paper](https://arxiv.org/abs/2006.12655)
- [Test Time Augmentation](https://arxiv.org/abs/2011.11156)
- [CUDA Programming Guide](https://docs.nvidia.com/cuda/)
