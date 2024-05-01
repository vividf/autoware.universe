// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_

#include "multi_object_tracker/tracker/motion_model/motion_model_base.hpp"

#include <Eigen/Core>
#include <kalman_filter/kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>

// cspell: ignore CTRV

class CTRVMotionModel : public MotionModel
{
private:
  // attributes
  rclcpp::Logger logger_;

  // motion parameters
  struct MotionParams
  {
    double q_cov_x;
    double q_cov_y;
    double q_cov_yaw;
    double q_cov_vel;
    double q_cov_wz;
    double max_vel;
    double max_wz;
  } motion_params_;

public:
  CTRVMotionModel();

  enum IDX { X = 0, Y = 1, YAW = 2, VEL = 3, WZ = 4 };
  const char DIM = 5;

  bool initialize(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
    const double & wz, const double & wz_cov);

  void setDefaultParams();

  void setMotionParams(
    const double & q_stddev_x, const double & q_stddev_y, const double & q_stddev_yaw,
    const double & q_stddev_vx, const double & q_stddev_wz);

  void setMotionLimits(const double & max_vel, const double & max_wz);

  bool updateStatePose(const double & x, const double & y, const std::array<double, 36> & pose_cov);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel, const std::array<double, 36> & twist_cov);

  bool adjustPosition(const double & x, const double & y);

  bool limitStates();

  bool predictStateStep(const double dt, KalmanFilter & ekf) const override;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__CTRV_MOTION_MODEL_HPP_
