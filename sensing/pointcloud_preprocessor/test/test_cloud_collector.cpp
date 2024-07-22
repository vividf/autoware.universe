// Copyright 2024 TIER IV, Inc.
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
// cloud_collector_test.cpp
// cloud_collector_test.cpp

// cloud_collector_test.cpp

#include "pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"
#include "pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <thread>

namespace pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent : public rclcpp::Node
{
public:
  PointCloudConcatenateDataSynchronizerComponent() : Node("test_node") {}

  void publishClouds()
  {
    // Mock implementation for testing purposes
  }
};

}  // namespace pointcloud_preprocessor

class CloudCollectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    concatenate_node_ =
      std::make_shared<pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent>();
    combine_cloud_handler_ = std::make_shared<pointcloud_preprocessor::CombineCloudHandler>(
      concatenate_node_.get(), std::vector<std::string>{"lidar_top", "lidar_left", "lidar_right"},
      "base_link", false, true);

    collector_ = std::make_shared<pointcloud_preprocessor::CloudCollector>(
      std::dynamic_pointer_cast<
        pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent>(
        concatenate_node_->shared_from_this()),
      collectors_, combine_cloud_handler_, 3, 1);

    collectors_.push_back(collector_);

    // Setup TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(concatenate_node_);
    tf_broadcaster_->sendTransform(generateStaticTransformMsg());

    // Spin the node for a while to ensure transforms are published
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(concatenate_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const std::string & parent_frame, const std::string & child_frame, double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  sensor_msgs::msg::PointCloud2 generatePointCloudMsg(
    bool generate_points, bool is_lidar_frame, std::string topic_name, rclcpp::Time stamp)
  {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.stamp = stamp;
    pointcloud_msg.header.frame_id = is_lidar_frame ? topic_name : "base_link";
    pointcloud_msg.height = 1;
    pointcloud_msg.is_dense = true;
    pointcloud_msg.is_bigendian = false;

    if (generate_points) {
      std::array<Eigen::Vector3f, number_of_points_> points = {{
        Eigen::Vector3f(10.0f, 0.0f, 0.0f),  // point 1
        Eigen::Vector3f(0.0f, 10.0f, 0.0f),  // point 2
        Eigen::Vector3f(0.0f, 0.0f, 10.0f),  // point 3
      }};

      sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
      modifier.setPointCloud2Fields(
        10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
        sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);

      modifier.resize(number_of_points_);

      sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(pointcloud_msg, "time_stamp");

      for (size_t i = 0; i < number_of_points_; ++i) {
        *iter_x = points[i].x();
        *iter_y = points[i].y();
        *iter_z = points[i].z();
        *iter_t = 0;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_t;
      }
    } else {
      pointcloud_msg.width = 0;
      pointcloud_msg.row_step = 0;
    }

    return pointcloud_msg;
  }

  std::vector<geometry_msgs::msg::TransformStamped> generateStaticTransformMsg()
  {
    // generate defined transformations
    return {
      generateTransformMsg("base_link", "lidar_top", 5.0, 5.0, 5.0, 0.683, 0.5, 0.183, 0.499),
      generateTransformMsg("base_link", "lidar_left", 1.0, 1.0, 3.0, 0.278, 0.717, 0.441, 0.453)};
    generateTransformMsg("base_link", "lidar_right", 1.0, 1.0, 3.0, 0.278, 0.717, 0.441, 0.453);
  }

  std::shared_ptr<pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent>
    concatenate_node_;
  std::list<std::shared_ptr<pointcloud_preprocessor::CloudCollector>> collectors_;
  std::shared_ptr<pointcloud_preprocessor::CombineCloudHandler> combine_cloud_handler_;
  std::shared_ptr<pointcloud_preprocessor::CloudCollector> collector_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  static constexpr int32_t timestamp_seconds_{10};
  static constexpr uint32_t timestamp_nanoseconds_{100000000};
  static constexpr size_t number_of_points_{3};
  bool debug_{true};
};

TEST_F(CloudCollectorTest, SetAndGetReferenceTimeStampBoundary)
{
  double reference_timestamp = 10.0;
  double noise_window = 0.1;
  collector_->setReferenceTimeStamp(reference_timestamp, noise_window);
  auto [min, max] = collector_->getReferenceTimeStampBoundary();
  EXPECT_DOUBLE_EQ(min, reference_timestamp - noise_window);
  EXPECT_DOUBLE_EQ(max, reference_timestamp + noise_window);
}

TEST_F(CloudCollectorTest, ProcessCloud)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 top_pointcloud =
    generatePointCloudMsg(true, false, "lidar_top", timestamp);
  collector_->processCloud(
    "lidar_top", std::make_shared<sensor_msgs::msg::PointCloud2>(top_pointcloud));
  EXPECT_EQ(collector_->topic_cloud_map_.size(), 1);
}

TEST_F(CloudCollectorTest, CombineClouds)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 top_pointcloud =
    generatePointCloudMsg(true, false, "lidar_top", timestamp);
  collector_->processCloud(
    "lidar_top", std::make_shared<sensor_msgs::msg::PointCloud2>(top_pointcloud));
  sensor_msgs::msg::PointCloud2 left_pointcloud =
    generatePointCloudMsg(true, false, "lidar_left", timestamp);
  collector_->processCloud(
    "lidar_left", std::make_shared<sensor_msgs::msg::PointCloud2>(left_pointcloud));
  sensor_msgs::msg::PointCloud2 right_pointcloud =
    generatePointCloudMsg(true, false, "lidar_right", timestamp);
  collector_->processCloud(
    "lidar_right", std::make_shared<sensor_msgs::msg::PointCloud2>(right_pointcloud));

  // This should trigger combineClouds
  collector_->combineClouds();

  // Validate that combinePointClouds was called
  auto concatenated_cloud = combine_cloud_handler_->getConcatenatePointcloud();
  EXPECT_NE(concatenated_cloud, nullptr);
}

TEST_F(CloudCollectorTest, DeleteCollector)
{
  collector_->deleteCollector();
  EXPECT_TRUE(collectors_.empty());
}

TEST_F(CloudCollectorTest, PrintTimer)
{
  // This test simply calls the printTimer method to ensure it runs without error
  collector_->printTimer();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

// sensor_msgs::PointCloud2ConstIterator<float> iter_x(*concatenated_cloud, "x");
// sensor_msgs::PointCloud2ConstIterator<float> iter_y(*concatenated_cloud, "y");
// sensor_msgs::PointCloud2ConstIterator<float> iter_z(*concatenated_cloud, "z");

// // Expected undistorted point cloud values
// std::array<Eigen::Vector3f, 3> expected_pointcloud = {
//   {Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 10.0f, 0.0f),
//     Eigen::Vector3f(0.0f, 0.0f, 10.0f)}};

// // Verify each point in the undistorted point cloud
// size_t i = 0;
// std::ostringstream oss;
// oss << "Expected pointcloud:\n";

// for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
//   oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
//   EXPECT_FLOAT_EQ(*iter_x, expected_pointcloud[i].x());
//   EXPECT_FLOAT_EQ(*iter_y, expected_pointcloud[i].y());
//   EXPECT_FLOAT_EQ(*iter_z, expected_pointcloud[i].z());
// }

// if (debug_) {
//   RCLCPP_INFO(concatenate_node_->get_logger(), "%s", oss.str().c_str());
// }
