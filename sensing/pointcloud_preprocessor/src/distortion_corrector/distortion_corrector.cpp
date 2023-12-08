// Copyright 2020 Tier IV, Inc.
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

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <deque>
#include <string>
#include <utility>
#include <time.h>

namespace pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "distortion_corrector");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Parameter
  time_stamp_field_name_ = declare_parameter("time_stamp_field_name", "time_stamp");
  use_imu_ = declare_parameter("use_imu", true);
  update_azimuth_and_distance_ = declare_parameter<bool>("update_azimuth_and_distance", true);

  // Publisher
  undistorted_points_pub_ =
    this->create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS());

  // Subscriber
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(
      &DistortionCorrectorComponent::onTwistWithCovarianceStamped, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", 10,
    std::bind(&DistortionCorrectorComponent::onImu, this, std::placeholders::_1));
  input_points_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::onPointCloud, this, std::placeholders::_1));
}

void DistortionCorrectorComponent::onTwistWithCovarianceStamped(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  tf2::Transform tf2_baselink_to_sensor_link{};
  getTransform(sensor_frame_, twist_msg->header.frame_id, &tf2_baselink_to_sensor_link);
  Eigen::Matrix4f eigen_baselink_to_sensor_link = tf2::transformToEigen(toMsg(tf2_baselink_to_sensor_link)).matrix().cast<float>();

  geometry_msgs::msg::Vector3 linear_velocity = twist_msg->twist.twist.linear;
  geometry_msgs::msg::Vector3 angular_velocity = twist_msg->twist.twist.angular;

  Eigen::Matrix3f rotation_matrix = eigen_baselink_to_sensor_link.topLeftCorner<3, 3>();
  Eigen::Vector3f translation_vector = eigen_baselink_to_sensor_link.topRightCorner<3, 1>();
  Sophus::SE3f T_b_to_s(rotation_matrix, translation_vector);
  Sophus::SE3f::Tangent twist(linear_velocity.x, linear_velocity.y, linear_velocity.z, angular_velocity.x, angular_velocity.y, angular_velocity.z);
  Sophus::SE3f::Tangent transformed_twist = T_b_to_s.Adj() * twist;

  linear_velocity.x = transformed_twist[0];
  linear_velocity.y = transformed_twist[1];
  linear_velocity.z = transformed_twist[2];
  angular_velocity.x = transformed_twist[3];
  angular_velocity.y = transformed_twist[4];
  angular_velocity.z = transformed_twist[5];
 
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist.linear = linear_velocity;
  msg.twist.angular = angular_velocity;
  twist_queue_.push_back(msg);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    }
    break;
  }
}

void DistortionCorrectorComponent::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  if (!use_imu_) {
    return;
  }

  tf2::Transform tf2_imu_link_to_sensor_link{};
  getTransform(sensor_frame_, imu_msg->header.frame_id, &tf2_imu_link_to_sensor_link);
  geometry_msgs::msg::TransformStamped::SharedPtr tf_imu2sensor_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf_imu2sensor_ptr->transform.rotation = tf2::toMsg(tf2_imu_link_to_sensor_link.getRotation());

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2sensor_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
      rclcpp::Time(imu_msg->header.stamp)) {
      angular_velocity_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue_.pop_front();
    }
    break;
  }
}

void DistortionCorrectorComponent::onPointCloud(PointCloud2::UniquePtr points_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_points_pub_->get_subscription_count() +
                                undistorted_points_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }


  double start = 0, finish = 0, duration = 0; 
  start = clock();
  undistortPointCloud(*points_msg);
  finish = clock();
  duration = (double)(finish - start) / CLOCKS_PER_SEC;
  std::cout << "duration: " << duration << std::endl;
  undistorted_points_pub_->publish(std::move(points_msg));

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

bool DistortionCorrectorComponent::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return false;
  }
  return true;
}

bool DistortionCorrectorComponent::undistortPointCloud(PointCloud2 & points)
{
  if (points.data.empty() || twist_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "input_pointcloud->points or twist_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(points.fields), std::cend(points.fields),
    [this](const sensor_msgs::msg::PointField & field) {
      return field.name == time_stamp_field_name_;
    });
  if (time_stamp_field_it == points.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }

  sensor_msgs::PointCloud2Iterator<float> it_x(points, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(points, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(points, "z");
  sensor_msgs::PointCloud2Iterator<float> it_azimuth(points, "azimuth");
  sensor_msgs::PointCloud2Iterator<float> it_distance(points, "distance");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(points, time_stamp_field_name_);

  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  auto twist_it = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  twist_it = twist_it == std::end(twist_queue_) ? std::end(twist_queue_) - 1 : twist_it;

  decltype(angular_velocity_queue_)::iterator imu_it;
  if (use_imu_ && !angular_velocity_queue_.empty()) {
    imu_it = std::lower_bound(
      std::begin(angular_velocity_queue_), std::end(angular_velocity_queue_),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    imu_it =
      imu_it == std::end(angular_velocity_queue_) ? std::end(angular_velocity_queue_) - 1 : imu_it;
  }

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  double twist_stamp = rclcpp::Time(twist_it->header.stamp).seconds();

  // For performance, instantiate outside of the for-loop
  Eigen::Vector4f point;
  Eigen::Vector4f undistorted_point;
  Eigen::Matrix4f transformation_matrix;
  Eigen::Matrix4f prev_transformation_matrix = Eigen::Matrix4f::Identity();



  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_azimuth, ++it_distance, ++it_time_stamp) {
    while (twist_it != std::end(twist_queue_) - 1 && *it_time_stamp > twist_stamp) {
      ++twist_it;
      twist_stamp = rclcpp::Time(twist_it->header.stamp).seconds();
    }

    float v_x{static_cast<float>(twist_it->twist.linear.x)};
    float v_y{static_cast<float>(twist_it->twist.linear.y)};
    float v_z{static_cast<float>(twist_it->twist.linear.z)};
    float w_x{static_cast<float>(twist_it->twist.angular.x)};
    float w_y{static_cast<float>(twist_it->twist.angular.y)};
    float w_z{static_cast<float>(twist_it->twist.angular.z)};
    
    /*
    float v{static_cast<float>(twist_it->twist.linear.x)};
    float w{static_cast<float>(twist_it->twist.angular.z)};
    */


    if (std::abs(*it_time_stamp - twist_stamp) > 0.1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 10000 /* ms */,
        "twist time_stamp is too late. Could not interpolate.");
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      w_x = 0.0f;
      w_y = 0.0f;
      w_z = 0.0f;
    }

    if (use_imu_ && !angular_velocity_queue_.empty()) {
      // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
      double imu_stamp = rclcpp::Time(imu_it->header.stamp).seconds();

      for (;
           (imu_it != std::end(angular_velocity_queue_) - 1 &&
            *it_time_stamp > rclcpp::Time(imu_it->header.stamp).seconds());
           ++imu_it) {
      }

      while (imu_it != std::end(angular_velocity_queue_) - 1 && *it_time_stamp > imu_stamp) {
        ++imu_it;
        imu_stamp = rclcpp::Time(imu_it->header.stamp).seconds();
      }

      if (std::abs(*it_time_stamp - imu_stamp) > 0.1) {
        RCLCPP_WARN_STREAM_THROTTLE(
          get_logger(), *get_clock(), 10000 /* ms */,
          "imu time_stamp is too late. Could not interpolate.");
      } else {
        w_x = static_cast<float>(imu_it->vector.x);
        w_y = static_cast<float>(imu_it->vector.y);
        w_z = static_cast<float>(imu_it->vector.z);
      }
    }

    float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);
    point << *it_x, *it_y, *it_z, 1.0;

    //std::cout << "\nbefore " << std::endl;
    //std::cout << "*it_x: " << *it_x << "*it_y: " << *it_y << "*it_z: " << *it_z << std::endl;

    Sophus::SE3f::Tangent twist(v_x, v_y, v_z, w_x, w_y, w_z);
    twist = twist * time_offset;
    transformation_matrix = Sophus::SE3f::exp(twist).matrix();

    transformation_matrix = transformation_matrix * prev_transformation_matrix;
    undistorted_point = transformation_matrix * point;
    
    *it_x = undistorted_point[0];
    *it_y = undistorted_point[1];
    *it_z = undistorted_point[2];

    //std::cout << "after " << std::endl;
    //std::cout << "*it_x: " << *it_x << "*it_y: " << *it_y << "*it_z: " << *it_z << std::endl;

    if (update_azimuth_and_distance_) {
      *it_distance = sqrt(*it_x * *it_x + *it_y * *it_y + *it_z * *it_z);
      *it_azimuth = cv::fastAtan2(*it_y, *it_x) * 100;
    }
    prev_time_stamp_sec = *it_time_stamp;
    prev_transformation_matrix = transformation_matrix;
  }
  return true;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistortionCorrectorComponent)
