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

#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cstdint>

// Package-internal arithmetic for builtin_interfaces::msg::Time. Message types are plain structs
// and fine to keep in the ROS-runtime-free core; what the core must not touch is rclcpp, so these
// minimal operators replace the rclcpp::Time helpers. This header is intentionally not exported
// (it lives under src/, not include/): it is an implementation detail of the concatenation core,
// not a public API.
namespace autoware::pointcloud_preprocessor
{

inline bool operator<(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  if (a.sec != b.sec) return a.sec < b.sec;
  return a.nanosec < b.nanosec;
}

inline bool operator>(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return b < a;
}

inline bool operator<=(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return !(b < a);
}

inline bool operator>=(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return !(a < b);
}

/// Advance a stamp by a duration (e.g. stamp + std::chrono::seconds(1)), for non-negative results.
inline builtin_interfaces::msg::Time operator+(
  const builtin_interfaces::msg::Time & stamp, const std::chrono::nanoseconds & duration)
{
  const int64_t total_nanoseconds =
    static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec + duration.count();
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(total_nanoseconds / 1'000'000'000LL);
  result.nanosec = static_cast<uint32_t>(total_nanoseconds % 1'000'000'000LL);
  return result;
}

/// Difference between two stamps, in integer nanoseconds. Converting a small difference to double
/// seconds afterwards keeps full nanosecond precision; subtracting two absolute double-second
/// stamps (~1.7e9) would lose a few hundred ns to floating-point rounding.
inline std::chrono::nanoseconds operator-(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return std::chrono::nanoseconds(
    (static_cast<int64_t>(a.sec) - b.sec) * 1'000'000'000LL +
    (static_cast<int64_t>(a.nanosec) - b.nanosec));
}

/// Absolute stamp in double seconds (matches rclcpp::Time::seconds()).
inline double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec) *
         1e-9;
}

}  // namespace autoware::pointcloud_preprocessor
