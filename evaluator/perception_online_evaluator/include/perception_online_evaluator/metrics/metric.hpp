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

#ifndef PERCEPTION_ONLINE_EVALUATOR__METRICS__METRIC_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__METRICS__METRIC_HPP_

#include "perception_online_evaluator/stat.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace perception_diagnostics
{
/**
 * @brief Enumeration of trajectory metrics
 */
enum class Metric {
  lateral_deviation,
  yaw_deviation,
  predicted_path_deviation,
  yaw_rate,
  SIZE,
};

using MetricStatMap = std::unordered_map<std::string, Stat<double>>;

static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"lateral_deviation", Metric::lateral_deviation},
  {"yaw_deviation", Metric::yaw_deviation},
  {"predicted_path_deviation", Metric::predicted_path_deviation},
  {"yaw_rate", Metric::yaw_rate}};

static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::lateral_deviation, "lateral_deviation"},
  {Metric::yaw_deviation, "yaw_deviation"},
  {Metric::predicted_path_deviation, "predicted_path_deviation"},
  {Metric::yaw_rate, "yaw_rate"}};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::lateral_deviation, "Lateral_deviation[m]"},
  {Metric::yaw_deviation, "Yaw_deviation[rad]"},
  {Metric::predicted_path_deviation, "Predicted_path_deviation[m]"},
  {Metric::yaw_rate, "Yaw_rate[rad/s]"}};

namespace details
{
static struct CheckCorrectMaps
{
  CheckCorrectMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metrics/metrics.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check;

}  // namespace details
}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__METRICS__METRIC_HPP_
