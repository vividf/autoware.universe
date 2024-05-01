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

#include "behavior_path_lane_change_module/interface.hpp"

#include "behavior_path_lane_change_module/utils/markers.hpp"
#include "behavior_path_lane_change_module/utils/utils.hpp"
#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "behavior_path_planner_common/marker_utils/utils.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace behavior_path_planner
{
using tier4_autoware_utils::appendMarkerArray;
using utils::lane_change::assignToCandidate;

LaneChangeInterface::LaneChangeInterface(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  std::unique_ptr<LaneChangeBase> && module_type)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},  // NOLINT
  parameters_{std::move(parameters)},
  module_type_{std::move(module_type)},
  prev_approved_path_{std::make_unique<PathWithLaneId>()}
{
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, name);
  logger_ = utils::lane_change::getLogger(module_type_->getModuleTypeStr());
}

void LaneChangeInterface::processOnEntry()
{
  waitApproval();
}

void LaneChangeInterface::processOnExit()
{
  module_type_->resetParameters();
  debug_marker_.markers.clear();
  post_process_safety_status_ = {};
  resetPathCandidate();
}

bool LaneChangeInterface::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  return module_type_->isLaneChangeRequired();
}

bool LaneChangeInterface::isExecutionReady() const
{
  return module_type_->isSafe() && !module_type_->isAbortState();
}

void LaneChangeInterface::updateData()
{
  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  module_type_->setPreviousDrivableAreaInfo(getPreviousModuleOutput().drivable_area_info);
  module_type_->setPreviousTurnSignalInfo(getPreviousModuleOutput().turn_signal_info);
  module_type_->updateSpecialData();

  if (isWaitingApproval()) {
    module_type_->updateLaneChangeStatus();
  }

  module_type_->resetStopPose();
  updateDebugMarker();
}

void LaneChangeInterface::postProcess()
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    const auto safety_status = module_type_->isApprovedPathSafe();
    post_process_safety_status_ =
      module_type_->evaluateApprovedPathWithUnsafeHysteresis(safety_status);
  }
  updateDebugMarker();
}

BehaviorModuleOutput LaneChangeInterface::plan()
{
  resetPathCandidate();
  resetPathReference();

  auto output = module_type_->generateOutput();
  path_reference_ = std::make_shared<PathWithLaneId>(output.reference_path);
  *prev_approved_path_ = getPreviousModuleOutput().path;

  stop_pose_ = module_type_->getStopPose();

  const auto & lane_change_debug = module_type_->getDebugData();
  for (const auto & [uuid, data] : lane_change_debug.collision_check_objects_after_approval) {
    const auto color = data.is_safe ? ColorName::GREEN : ColorName::RED;
    setObjectsOfInterestData(data.current_obj_pose, data.obj_shape, color);
  }

  updateSteeringFactorPtr(output);
  if (module_type_->isAbortState()) {
    waitApproval();
    removeRTCStatus();
    const auto candidate = planCandidate();
    path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
    updateRTCStatus(
      candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change);
  } else {
    clearWaitingApproval();
  }

  return output;
}

BehaviorModuleOutput LaneChangeInterface::planWaitingApproval()
{
  *prev_approved_path_ = getPreviousModuleOutput().path;

  BehaviorModuleOutput out;
  out = module_type_->getTerminalLaneChangePath();
  module_type_->insertStopPoint(module_type_->getLaneChangeStatus().current_lanes, out.path);
  out.turn_signal_info =
    getCurrentTurnSignalInfo(out.path, getPreviousModuleOutput().turn_signal_info);

  const auto & lane_change_debug = module_type_->getDebugData();
  for (const auto & [uuid, data] : lane_change_debug.collision_check_objects) {
    const auto color = data.is_safe ? ColorName::GREEN : ColorName::RED;
    setObjectsOfInterestData(data.current_obj_pose, data.obj_shape, color);
  }

  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
  stop_pose_ = module_type_->getStopPose();

  if (!module_type_->isValidPath()) {
    removeRTCStatus();
    path_candidate_ = std::make_shared<PathWithLaneId>();
    return out;
  }

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  updateRTCStatus(
    candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change);
  is_abort_path_approved_ = false;

  return out;
}

CandidateOutput LaneChangeInterface::planCandidate() const
{
  const auto selected_path = module_type_->getLaneChangePath();

  if (selected_path.path.points.empty()) {
    return {};
  }

  CandidateOutput output = assignToCandidate(selected_path, module_type_->getEgoPosition());

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

void LaneChangeInterface::updateModuleParams(const std::any & parameters)
{
  parameters_ = std::any_cast<std::shared_ptr<LaneChangeParameters>>(parameters);
}

void LaneChangeInterface::setData(const std::shared_ptr<const PlannerData> & data)
{
  planner_data_ = data;
  module_type_->setData(data);
}

bool LaneChangeInterface::canTransitSuccessState()
{
  auto log_debug_throttled = [&](std::string_view message) -> void {
    RCLCPP_DEBUG(getLogger(), "%s", message.data());
  };
  updateDebugMarker();

  if (module_type_->specialExpiredCheck() && isWaitingApproval()) {
    log_debug_throttled("Run specialExpiredCheck.");
    if (isWaitingApproval()) {
      return true;
    }
  }

  if (module_type_->isAbortState() && module_type_->hasFinishedAbort()) {
    log_debug_throttled("Abort process has completed.");
    return true;
  }

  if (module_type_->hasFinishedLaneChange()) {
    module_type_->resetParameters();
    log_debug_throttled("Lane change process has completed.");
    return true;
  }

  log_debug_throttled("Lane changing process is ongoing");
  return false;
}

bool LaneChangeInterface::canTransitFailureState()
{
  auto log_debug_throttled = [&](std::string_view message) -> void {
    RCLCPP_DEBUG(getLogger(), "%s", message.data());
  };

  updateDebugMarker();
  log_debug_throttled(__func__);

  if (module_type_->isAbortState() && !module_type_->hasFinishedAbort()) {
    log_debug_throttled("Abort process has on going.");
    return false;
  }

  if (isWaitingApproval()) {
    log_debug_throttled("Can't transit to failure state. Module is WAITING_FOR_APPROVAL");
    return false;
  }

  if (module_type_->isCancelEnabled() && module_type_->isEgoOnPreparePhase()) {
    if (module_type_->isStoppedAtRedTrafficLight()) {
      log_debug_throttled("Stopping at traffic light while in prepare phase. Cancel lane change");
      module_type_->toCancelState();
      return true;
    }

    if (post_process_safety_status_.is_safe) {
      log_debug_throttled("Can't transit to failure state. Ego is on prepare, and it's safe.");
      return false;
    }

    if (module_type_->isAbleToReturnCurrentLane()) {
      log_debug_throttled("It's possible to return to current lane. Cancel lane change.");
      return true;
    }
  }

  if (post_process_safety_status_.is_safe) {
    log_debug_throttled("Can't transit to failure state. Ego is lane changing, and it's safe.");
    return false;
  }

  if (module_type_->isRequiredStop(post_process_safety_status_.is_object_coming_from_rear)) {
    log_debug_throttled("Module require stopping");
  }

  if (!module_type_->isCancelEnabled()) {
    log_debug_throttled(
      "Lane change path is unsafe but cancel was not enabled. Continue lane change.");
    return false;
  }

  if (!module_type_->isAbortEnabled()) {
    log_debug_throttled(
      "Lane change path is unsafe but abort was not enabled. Continue lane change.");
    return false;
  }

  if (!module_type_->isAbleToReturnCurrentLane()) {
    log_debug_throttled("It's is not possible to return to original lane. Continue lane change.");
    return false;
  }

  const auto found_abort_path = module_type_->calcAbortPath();
  if (!found_abort_path) {
    log_debug_throttled(
      "Lane change path is unsafe but abort path not found. Continue lane change.");
    return false;
  }

  log_debug_throttled("Lane change path is unsafe. Abort lane change.");
  module_type_->toAbortState();
  return false;
}

void LaneChangeInterface::updateDebugMarker() const
{
  debug_marker_.markers.clear();
  if (!parameters_->publish_debug_marker) {
    return;
  }
  using marker_utils::lane_change_markers::createDebugMarkerArray;
  debug_marker_ = createDebugMarkerArray(module_type_->getDebugData(), module_type_->getEgoPose());
}

MarkerArray LaneChangeInterface::getModuleVirtualWall()
{
  using marker_utils::lane_change_markers::createLaneChangingVirtualWallMarker;
  MarkerArray marker;

  if (!parameters_->publish_debug_marker) {
    return marker;
  }

  if (isWaitingApproval() || getCurrentStatus() != ModuleStatus::RUNNING) {
    return marker;
  }
  const auto & start_pose = module_type_->getLaneChangePath().info.lane_changing_start;
  const auto start_marker =
    createLaneChangingVirtualWallMarker(start_pose, name(), clock_->now(), "lane_change_start");

  const auto & end_pose = module_type_->getLaneChangePath().info.lane_changing_end;
  const auto end_marker =
    createLaneChangingVirtualWallMarker(end_pose, name(), clock_->now(), "lane_change_end");
  marker.markers.reserve(start_marker.markers.size() + end_marker.markers.size());
  appendMarkerArray(start_marker, &marker);
  appendMarkerArray(end_marker, &marker);
  return marker;
}

void LaneChangeInterface::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto steering_factor_direction = std::invoke([&]() {
    if (module_type_->getDirection() == Direction::LEFT) {
      return SteeringFactor::LEFT;
    }
    if (module_type_->getDirection() == Direction::RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::UNKNOWN;
  });

  const auto current_position = module_type_->getEgoPosition();
  const auto status = module_type_->getLaneChangeStatus();
  const auto start_distance = motion_utils::calcSignedArcLength(
    output.path.points, current_position, status.lane_change_path.info.shift_line.start.position);
  const auto finish_distance = motion_utils::calcSignedArcLength(
    output.path.points, current_position, status.lane_change_path.info.shift_line.end.position);

  steering_factor_interface_ptr_->updateSteeringFactor(
    {status.lane_change_path.info.shift_line.start, status.lane_change_path.info.shift_line.end},
    {start_distance, finish_distance}, PlanningBehavior::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void LaneChangeInterface::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.info.shift_line.start, selected_path.info.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    PlanningBehavior::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}

TurnSignalInfo LaneChangeInterface::getCurrentTurnSignalInfo(
  const PathWithLaneId & path, const TurnSignalInfo & original_turn_signal_info)
{
  const auto & current_lanes = module_type_->getLaneChangeStatus().current_lanes;
  const auto & is_valid = module_type_->getLaneChangeStatus().is_valid_path;
  const auto & lane_change_path = module_type_->getLaneChangeStatus().lane_change_path;
  const auto & lane_change_param = module_type_->getLaneChangeParam();

  if (
    module_type_->getModuleType() != LaneChangeModuleType::NORMAL || current_lanes.empty() ||
    !is_valid) {
    return original_turn_signal_info;
  }

  // check direction
  TurnSignalInfo current_turn_signal_info;
  const auto & current_pose = module_type_->getEgoPose();
  const auto direction = module_type_->getDirection();
  if (direction == Direction::LEFT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  } else if (direction == Direction::RIGHT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  }

  if (path.points.empty()) {
    current_turn_signal_info.desired_start_point = current_pose;
    current_turn_signal_info.required_start_point = current_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_end_point = lane_change_path.info.lane_changing_end;
    return current_turn_signal_info;
  }

  const auto & min_length_for_turn_signal_activation =
    lane_change_param.min_length_for_turn_signal_activation;
  const auto & route_handler = module_type_->getRouteHandler();
  const auto & common_parameter = module_type_->getCommonParam();
  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double next_lane_change_buffer =
    utils::lane_change::calcMinimumLaneChangeLength(lane_change_param, shift_intervals);
  const double & nearest_dist_threshold = common_parameter.ego_nearest_dist_threshold;
  const double & nearest_yaw_threshold = common_parameter.ego_nearest_yaw_threshold;
  const double & base_to_front = common_parameter.base_link2front;

  const double buffer =
    next_lane_change_buffer + min_length_for_turn_signal_activation + base_to_front;
  const double path_length = motion_utils::calcArcLength(path.points);
  const size_t & current_nearest_seg_idx =
    motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double dist_to_terminal = utils::getDistanceToEndOfLane(current_pose, current_lanes);
  const auto start_pose =
    motion_utils::calcLongitudinalOffsetPose(path.points, 0, std::max(path_length - buffer, 0.0));
  if (dist_to_terminal - base_to_front < buffer && start_pose) {
    // modify turn signal
    current_turn_signal_info.desired_start_point = *start_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_start_point = current_turn_signal_info.desired_start_point;
    current_turn_signal_info.required_end_point = current_turn_signal_info.desired_end_point;

    const auto & original_command = original_turn_signal_info.turn_signal.command;
    if (
      original_command == TurnIndicatorsCommand::DISABLE ||
      original_command == TurnIndicatorsCommand::NO_COMMAND) {
      return current_turn_signal_info;
    }

    // check the priority of turn signals
    return module_type_->getTurnSignalDecider().use_prior_turn_signal(
      path, current_pose, current_nearest_seg_idx, original_turn_signal_info,
      current_turn_signal_info, nearest_dist_threshold, nearest_yaw_threshold);
  }

  // not in the vicinity of the end of the path. return original
  return original_turn_signal_info;
}
}  // namespace behavior_path_planner
