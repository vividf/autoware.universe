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
// minimal helpers replace the rclcpp::Time ones. Named functions and a comparator on purpose, not
// operator overloads: operators on a foreign type are only found by unqualified lookup from this
// namespace (std::greater etc. instantiated inside std would not see them) and risk ODR clashes
// with other packages doing the same. This header is intentionally not exported (it lives under
// src/, not include/): it is an implementation detail of the concatenation core, not a public API.
namespace autoware::pointcloud_preprocessor
{

/// Strict weak ordering for stamps, usable as a Compare with the std algorithms.
struct TimeLess
{
  bool operator()(
    const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b) const
  {
    if (a.sec != b.sec) return a.sec < b.sec;
    return a.nanosec < b.nanosec;
  }
};

/// Named instance so direct comparisons read like a predicate: is_earlier(a, b) means "a < b".
inline constexpr TimeLess is_earlier{};

/// Advance a stamp by a (possibly negative) duration, e.g. add(stamp, -1s). The result is floor-
/// normalized so nanosec stays in [0, 1e9) even when the result is negative (negative sec), per
/// the ROS Time convention.
inline builtin_interfaces::msg::Time add(
  const builtin_interfaces::msg::Time & stamp, const std::chrono::nanoseconds & duration)
{
  const int64_t total_nanoseconds =
    static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec + duration.count();
  int64_t sec = total_nanoseconds / 1'000'000'000LL;
  int64_t nanosec = total_nanoseconds % 1'000'000'000LL;
  if (nanosec < 0) {
    sec -= 1;
    nanosec += 1'000'000'000LL;
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(sec);
  result.nanosec = static_cast<uint32_t>(nanosec);
  return result;
}

/// Difference between two stamps, in integer nanoseconds. Converting a small difference to double
/// seconds afterwards keeps full nanosecond precision; subtracting two absolute double-second
/// stamps (~1.7e9) would lose a few hundred ns to floating-point rounding.
inline std::chrono::nanoseconds subtract(
  const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return std::chrono::nanoseconds(
    (static_cast<int64_t>(a.sec) - b.sec) * 1'000'000'000LL +
    (static_cast<int64_t>(a.nanosec) - b.nanosec));
}

/// Absolute stamp in double seconds. Computed as sec + nanosec * 1e-9 so the value is rounded only
/// once (at the final addition); converting a total int64 nanosecond count (~1.7e18, beyond
/// double's 2^53 integer range) to double first would round twice and can be off by a few hundred
/// nanoseconds more.
inline double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace autoware::pointcloud_preprocessor
