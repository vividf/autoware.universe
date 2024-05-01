// Copyright 2023 TIER IV, Inc.
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

#include "image_projection_based_fusion/segmentation_pointcloud_fusion/node.hpp"

#include "image_projection_based_fusion/utils/geometry.hpp"
#include "image_projection_based_fusion/utils/utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace image_projection_based_fusion
{
SegmentPointCloudFusionNode::SegmentPointCloudFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<PointCloud2, PointCloud2, Image>("segmentation_pointcloud_fusion", options)
{
  filter_distance_threshold_ = declare_parameter<float>("filter_distance_threshold");
  filter_semantic_label_target_ =
    declare_parameter<std::vector<bool>>("filter_semantic_label_target");
}

void SegmentPointCloudFusionNode::preprocess(__attribute__((unused)) PointCloud2 & pointcloud_msg)
{
  return;
}

void SegmentPointCloudFusionNode::postprocess(__attribute__((unused)) PointCloud2 & pointcloud_msg)
{
  return;
}
void SegmentPointCloudFusionNode::fuseOnSingleImage(
  const PointCloud2 & input_pointcloud_msg, __attribute__((unused)) const std::size_t image_id,
  [[maybe_unused]] const Image & input_mask, __attribute__((unused)) const CameraInfo & camera_info,
  __attribute__((unused)) PointCloud2 & output_pointcloud_msg)
{
  if (input_pointcloud_msg.data.empty()) {
    return;
  }
  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(
      std::make_shared<sensor_msgs::msg::Image>(input_mask), input_mask.encoding);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:%s", e.what());
    return;
  }

  cv::Mat mask = in_image_ptr->image;
  if (mask.cols == 0 || mask.rows == 0) {
    return;
  }
  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2), camera_info.p.at(3),
    camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6), camera_info.p.at(7),
    camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), camera_info.p.at(11), 0.0, 0.0,
    0.0, 1.0;
  geometry_msgs::msg::TransformStamped transform_stamped;
  // transform pointcloud from frame id to camera optical frame id
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, input_mask.header.frame_id, input_pointcloud_msg.header.frame_id,
      input_pointcloud_msg.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  PointCloud2 transformed_cloud;
  tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

  PointCloud output_cloud;

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x"),
       iter_y(transformed_cloud, "y"), iter_z(transformed_cloud, "z"),
       iter_orig_x(input_pointcloud_msg, "x"), iter_orig_y(input_pointcloud_msg, "y"),
       iter_orig_z(input_pointcloud_msg, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_orig_x, ++iter_orig_y, ++iter_orig_z) {
    // skip filtering pointcloud behind the camera or too far from camera
    if (*iter_z <= 0.0 || *iter_z > filter_distance_threshold_) {
      output_cloud.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
      continue;
    }

    Eigen::Vector4d projected_point = projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

    bool is_inside_image =
      normalized_projected_point.x() > 0 && normalized_projected_point.x() < camera_info.width &&
      normalized_projected_point.y() > 0 && normalized_projected_point.y() < camera_info.height;
    if (!is_inside_image) {
      output_cloud.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
      continue;
    }

    // skip filtering pointcloud where semantic id out of the defined list
    uint8_t semantic_id = mask.at<uint8_t>(
      static_cast<uint16_t>(normalized_projected_point.y()),
      static_cast<uint16_t>(normalized_projected_point.x()));
    if (static_cast<size_t>(semantic_id) >= filter_semantic_label_target_.size()) {
      output_cloud.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
      continue;
    }
    if (!filter_semantic_label_target_.at(semantic_id)) {
      output_cloud.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
    }
  }

  pcl::toROSMsg(output_cloud, output_pointcloud_msg);
  output_pointcloud_msg.header = input_pointcloud_msg.header;
}

bool SegmentPointCloudFusionNode::out_of_scope(__attribute__((unused))
                                               const PointCloud2 & filtered_cloud)
{
  return false;
}
}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::SegmentPointCloudFusionNode)
