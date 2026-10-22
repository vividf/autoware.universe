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

// Unit tests for the ROS-runtime-free CloudConcatenator: the collector lifecycle
// (matching, completion, arrival-driven timeout, flush). No rclcpp is initialized anywhere here --
// the whole pipeline runs deterministically from caller-provided arrival times.

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_concatenator.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::pointcloud_preprocessor::ConcatenatedFrameStatus;
using autoware::pointcloud_preprocessor::MatchingStrategyType;
using autoware::pointcloud_preprocessor::parse_matching_strategy;
using CloudConcatenator =
  autoware::pointcloud_preprocessor::CloudConcatenator<sensor_msgs::msg::PointCloud2>;

const std::vector<std::string> kInputTopics = {"lidar_top", "lidar_left", "lidar_right"};
constexpr char kOutputFrame[] = "base_link";
constexpr std::size_t kPointsPerCloud = 3;

// Advanced strategy config (mirrors the node parameters), one entry per input topic.
const std::vector<double> kOffsets = {0.0, 0.04, 0.08};
const std::vector<double> kNoise = {0.01, 0.01, 0.01};

// Minimal x/y/z FLOAT32 cloud in the output frame (so no extrinsic is needed);
// convert_to_xyzirc_cloud upgrades it to the XYZIRC layout internally.
sensor_msgs::msg::PointCloud2::ConstSharedPtr make_cloud(
  int32_t sec, uint32_t nanosec, const char * frame_id = kOutputFrame)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp.sec = sec;
  cloud->header.stamp.nanosec = nanosec;
  cloud->header.frame_id = frame_id;
  cloud->height = 1;
  cloud->width = kPointsPerCloud;

  auto field = [](const char * name, uint32_t offset) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    return f;
  };
  cloud->fields = {field("x", 0), field("y", 4), field("z", 8)};
  cloud->is_bigendian = false;
  cloud->point_step = 12;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->is_dense = true;

  cloud->data.resize(cloud->row_step);
  const float points[kPointsPerCloud][3] = {{10.f, 0.f, 1.f}, {0.f, 10.f, 2.f}, {0.f, 0.f, 10.f}};
  std::memcpy(cloud->data.data(), points, sizeof(points));
  return cloud;
}

CloudConcatenator make_naive_concatenator(double timeout_sec = 0.2)
{
  return CloudConcatenator(
    kInputTopics, kOutputFrame, timeout_sec, /*is_motion_compensated=*/false,
    /*publish_synchronized_pointcloud=*/false,
    /*keep_input_frame_in_synchronized_pointcloud=*/false, MatchingStrategyType::naive);
}

CloudConcatenator make_advanced_concatenator(double timeout_sec = 0.2)
{
  return CloudConcatenator(
    kInputTopics, kOutputFrame, timeout_sec, /*is_motion_compensated=*/false,
    /*publish_synchronized_pointcloud=*/false,
    /*keep_input_frame_in_synchronized_pointcloud=*/false, MatchingStrategyType::advanced, kOffsets,
    kNoise);
}

}  // namespace

TEST(CloudConcatenatorConstruction, UnknownStrategyNameIsRejectedAtParse)
{
  // Strategy names are validated once, at the string boundary; the concatenator itself only
  // accepts the typed enum.
  EXPECT_THROW(parse_matching_strategy("no_such_strategy"), std::invalid_argument);
}

TEST(CloudConcatenatorConstruction, AdvancedRequiresOffsetsAndNoiseWindow)
{
  EXPECT_THROW(
    CloudConcatenator(
      kInputTopics, kOutputFrame, 0.2, false, false, false, MatchingStrategyType::advanced),
    std::invalid_argument);
}

TEST(CloudConcatenatorConstruction, AdvancedRejectsMismatchedSizes)
{
  // Size validation is delegated to AdvancedMatchingPolicy (shared with the node).
  EXPECT_THROW(
    CloudConcatenator(
      kInputTopics, kOutputFrame, 0.2, false, false, false, MatchingStrategyType::advanced,
      std::vector<double>{0.0, 0.04}, kNoise),
    std::runtime_error);
}

TEST(CloudConcatenator, ProcessCloudRejectsUnknownTopic)
{
  auto concatenator = make_naive_concatenator();
  EXPECT_THROW(
    static_cast<void>(
      concatenator.process_cloud("not_a_configured_topic", make_cloud(10, 0), 100.0)),
    std::invalid_argument);
}

TEST(CloudConcatenator, NaiveCompletesWhenAllTopicsArrive)
{
  auto concatenator = make_naive_concatenator();

  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.00).empty());
  EXPECT_TRUE(concatenator.process_cloud("lidar_left", make_cloud(10, 1'000'000), 100.01).empty());
  const auto frames = concatenator.process_cloud("lidar_right", make_cloud(10, 2'000'000), 100.02);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kComplete);
  ASSERT_TRUE(frames[0].result.concatenate_cloud_ptr != nullptr);
  EXPECT_EQ(frames[0].result.concatenate_cloud_ptr->width, kInputTopics.size() * kPointsPerCloud);
  EXPECT_EQ(frames[0].result.concatenate_cloud_ptr->header.frame_id, kOutputFrame);
}

TEST(CloudConcatenator, NaiveTimesOutIncompleteCollector)
{
  auto concatenator = make_naive_concatenator(0.2);

  // Only lidar_top; a cloud arriving 0.5 s later (> 0.2 s timeout) closes the stale collector.
  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0).empty());
  const auto frames = concatenator.process_cloud("lidar_left", make_cloud(10, 0), 100.5);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kTimeout);
  EXPECT_EQ(frames[0].result.concatenate_cloud_ptr->width, kPointsPerCloud);  // only lidar_top
}

TEST(CloudConcatenator, CloseExpiredCollectorsWithoutACloud)
{
  // The node-adoption hook: a timer drives the clock while no clouds arrive.
  auto concatenator = make_naive_concatenator(0.2);
  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0).empty());

  EXPECT_TRUE(concatenator.close_expired_collectors(100.1).empty());  // not expired yet

  const auto frames = concatenator.close_expired_collectors(100.5);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kTimeout);
  EXPECT_EQ(frames[0].first_arrival_time, 100.0);
}

TEST(CloudConcatenator, ClockGoingBackwardsClosesCollectorsOpenedBeforeIt)
{
  // Under use_sim_time a restarted rosbag moves the node's clock backwards. A collector opened
  // before the jump must still close instead of waiting for its old deadline to come around again.
  auto concatenator = make_advanced_concatenator(0.2);
  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0).empty());

  const auto frames = concatenator.close_expired_collectors(50.0);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kTimeout);
  EXPECT_FALSE(concatenator.next_deadline().has_value());
}

TEST(CloudConcatenator, ClockGoingBackwardsKeepsLiveCollectorsOverStaleOnes)
{
  // Two partial frames are open when the bag restarts. The restarted stream carries earlier stamps,
  // so drop-oldest would evict its collectors rather than the stale ones.
  CloudConcatenator concatenator(
    kInputTopics, kOutputFrame, /*timeout_sec=*/0.2, false, false, false,
    MatchingStrategyType::advanced, kOffsets, kNoise, /*max_open_collectors=*/3);
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0));
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(10, 500'000'000), 100.1));

  // After the jump, frame N+1 opens before frame N's other clouds arrive.
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(5, 0), 50.0));
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(5, 100'000'000), 50.1));
  static_cast<void>(concatenator.process_cloud("lidar_left", make_cloud(5, 40'000'000), 50.14));
  const auto frames = concatenator.process_cloud("lidar_right", make_cloud(5, 80'000'000), 50.18);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kComplete);
  EXPECT_DOUBLE_EQ(frames[0].reference_time, 5.0);
  EXPECT_EQ(concatenator.stats().force_dropped_collectors, 0u);
}

TEST(CloudConcatenator, FlushEmitsOpenCollectorsOldestFirst)
{
  auto concatenator = make_advanced_concatenator(10.0);  // long timeout: nothing expires

  // lidar_left's corrected stamp (10.16) is far outside lidar_top's window (~[9.99, 10.01]), so it
  // opens a second collector instead of joining the first.
  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.00).empty());
  EXPECT_TRUE(
    concatenator.process_cloud("lidar_left", make_cloud(10, 200'000'000), 100.05).empty());

  const auto frames = concatenator.flush();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kTimeout);
  EXPECT_EQ(frames[1].status, ConcatenatedFrameStatus::kTimeout);
  EXPECT_LT(frames[0].first_arrival_time, frames[1].first_arrival_time);  // oldest first
}

TEST(CloudConcatenator, AdvancedMatchesOffsetCorrectedTimestamps)
{
  auto concatenator = make_advanced_concatenator();

  // Per-topic stamps staggered by exactly the offsets: corrected timestamps all map to 10.00 s.
  EXPECT_TRUE(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.00).empty());
  EXPECT_TRUE(concatenator.process_cloud("lidar_left", make_cloud(10, 40'000'000), 100.01).empty());
  const auto frames = concatenator.process_cloud("lidar_right", make_cloud(10, 80'000'000), 100.02);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].status, ConcatenatedFrameStatus::kComplete);
  EXPECT_EQ(frames[0].result.concatenate_cloud_ptr->width, kInputTopics.size() * kPointsPerCloud);
  // The frame carries the matching context of its collector.
  EXPECT_DOUBLE_EQ(frames[0].reference_time, 10.0);
  EXPECT_DOUBLE_EQ(frames[0].noise_window, kNoise[0]);
  EXPECT_DOUBLE_EQ(frames[0].first_arrival_time, 100.00);
  ASSERT_TRUE(frames[0].result.concatenation_info_ptr != nullptr);
}

TEST(CloudConcatenator, NextDeadlineTracksOldestOpenCollector)
{
  auto concatenator = make_naive_concatenator(0.2);
  EXPECT_FALSE(concatenator.next_deadline().has_value());  // nothing open

  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0));
  ASSERT_TRUE(concatenator.next_deadline().has_value());
  EXPECT_DOUBLE_EQ(*concatenator.next_deadline(), 100.2);

  static_cast<void>(concatenator.close_expired_collectors(100.5));
  EXPECT_FALSE(concatenator.next_deadline().has_value());  // closed again
}

TEST(CloudConcatenator, MaxOpenCollectorsDropsOldestUnpublished)
{
  // Cap of 2 open collectors (the node uses its historical pool size); a third collector must
  // evict the one with the oldest reference without emitting it.
  CloudConcatenator concatenator(
    kInputTopics, kOutputFrame, /*timeout_sec=*/10.0, false, false, false,
    MatchingStrategyType::advanced, kOffsets, kNoise, /*max_open_collectors=*/2);

  // Three lidar_top clouds with stamps far outside each other's windows -> three collectors.
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(10, 0), 100.0));
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(11, 0), 100.1));
  static_cast<void>(concatenator.process_cloud("lidar_top", make_cloud(12, 0), 100.2));

  EXPECT_EQ(concatenator.stats().force_dropped_collectors, 1u);

  // The survivors are the two newest collectors; flushing yields them oldest-first.
  const auto frames = concatenator.flush();
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_DOUBLE_EQ(frames[0].reference_time, 11.0);
  EXPECT_DOUBLE_EQ(frames[1].reference_time, 12.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
