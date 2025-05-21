// Copyright 2022 TIER IV, Inc.
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

#include "image_diagnostics_node.hpp"

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::image_diagnostics
{
using image_diagnostics::Image_State;
ImageDiagNode::ImageDiagNode(const rclcpp::NodeOptions & node_options)
: Node("image_diagnostics_node", node_options)
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "input/raw_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageDiagNode::ImageChecker, this, std::placeholders::_1));
  block_diag_image_pub_ =
    image_transport::create_publisher(this, "image_diag/debug/diag_block_image");
  dft_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/dft_image");
  gray_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/gray_image");

  image_state_pub_ = create_publisher<tier4_debug_msgs::msg::Int32Stamped>(
    "image_diag/image_state_diag", rclcpp::SensorDataQoS());

  updater_.setHardwareID("Image_Diagnostics");
  updater_.add(
    std::string(this->get_namespace()) + ": Image_Diagnostics", this,
    &ImageDiagNode::onImageDiagChecker);
  updater_.setPeriod(0.1);
}

void ImageDiagNode::onImageDiagChecker(DiagnosticStatusWrapper & stat)
{
  const cv::Mat gray_img = preprocess_image(input_image_msg);
  const RegionFeatures features = compute_image_features(gray_img);
  const std::vector<Image_State> region_states = classify_regions(features);

  auto level = DiagnosticStatusWrapper::OK;
  std::string msg = "OK";

  if (params_.diagnostic_status < 0) {
    level = DiagnosticStatusWrapper::STALE;
    msg = "STALE";
  } else if (params_.diagnostic_status == 1) {
    level = DiagnosticStatusWrapper::WARN;
    msg = "WARNING: abnormal state in image diagnostics";
  } else if (params_.diagnostic_status == 2) {
    level = DiagnosticStatusWrapper::ERROR;
    msg = "ERROR: abnormal state in image diagnostics";
  }

  stat.summary(level, msg);
}

void ImageDiagNode::ImageChecker(const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg)
{
  cv::Mat img_gray;
  cv::Mat img_gray_32b;
  cv::Mat img_gray_blockage_bin;
  cv::Mat tmp;

  img_gray = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  cv::Size size;
  size.height = params_.image_resize_height;
  size.width = static_cast<int>((img_gray.cols * size.height) / img_gray.rows);
  cv::resize(img_gray, img_gray, size);
  int block_size_h = std::floor(img_gray.cols / params_.number_block_horizontal);
  int block_size_v = std::floor(img_gray.rows / params_.number_block_vertical);
  int region_pix_count = block_size_h * block_size_v;

  int block_size_h_dft = cv::getOptimalDFTSize(block_size_h);
  int block_size_v_dft = cv::getOptimalDFTSize(block_size_v);

  std::vector<int> region_average_vec;
  std::vector<float> region_blockage_ratio_vec;
  std::vector<float> region_freq_sum_vec;
  cv::threshold(
    img_gray, img_gray_blockage_bin, params_.blockage_intensity_thresh, 255, cv::THRESH_BINARY);
  img_gray.convertTo(img_gray_32b, CV_32FC1);
  cv::Mat imgDCT(size, CV_32FC1);
  imgDCT.setTo(0.0);
  cv::Mat imgDFT(size, CV_32FC1);
  imgDFT.setTo(0.0);
  // calculate the features of each small block in image
  for (int v = 0; v < params_.number_block_vertical; v++) {
    for (int h = 0; h < params_.number_block_horizontal; h++) {
      int x = h * block_size_h;
      int y = v * block_size_v;
      cv::Rect roi(x, y, block_size_h, block_size_v);
      int intensity_average = static_cast<int>(cv::mean(img_gray(roi))[0]);
      int blockage_pix_num = cv::countNonZero(img_gray_blockage_bin(roi));
      float roi_blockage_ratio =
        (static_cast<float>(region_pix_count) - static_cast<float>(blockage_pix_num)) /
        static_cast<float>(region_pix_count);
      cv::copyMakeBorder(
        img_gray_32b(roi), img_gray_32b(roi), 0, block_size_v_dft - img_gray_32b(roi).rows, 0,
        block_size_h_dft - img_gray_32b(roi).cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
      std::vector<cv::Mat> channelImg{
        img_gray_32b(roi), cv::Mat::zeros(img_gray_32b(roi).size(), CV_32FC1)};
      cv::Mat srcComp, freqComp;
      cv::merge(channelImg, srcComp);
      cv::dft(srcComp, freqComp);
      shiftImage(freqComp);
      cv::split(freqComp, channelImg);
      cv::Rect original_roi(0, 0, block_size_h, block_size_v);
      channelImg[0](original_roi)
        .copyTo(imgDFT(cv::Rect(roi.x, roi.y, block_size_h, block_size_v)));

      cv::Mat rect_tmp = img_gray_32b(roi);
      channelImg[0](original_roi).copyTo(rect_tmp);
      cv::log(rect_tmp, rect_tmp);
      const float region_freq_average = cv::mean(rect_tmp)[0];

      region_average_vec.push_back(intensity_average);
      region_blockage_ratio_vec.push_back(roi_blockage_ratio);
      region_freq_sum_vec.push_back(region_freq_average);
    }
  }

  std::vector<ImageDiagNode::Image_State> ImageDiagNode::classify_regions(
    const RegionFeatures & features) const
  {
    std::vector<Image_State> states;
    for (size_t i = 0; i < features.avg_intensity.size(); ++i) {
      Image_State state = Image_State::NORMAL;
      if (features.avg_intensity[i] < params_.shadow_intensity_threshold) {
        state = Image_State::DARK;
      } else if (
        region_blockage_ratio_vec[region] > params_.blockage_ratio_thresh &&
        region_freq_sum_vec[region] < params_.blockage_freq_ratio_thresh) {
        region_state = Image_State::BLOCKAGE;
      } else if (
        features.freq_sum[i] < params_.low_visibility_frequency_threshold &&
        features.avg_intensity[i] < params_.highlight_intensity_threshold) {
        state = Image_State::LOW_VIS;
      } else if (features.avg_intensity[i] > params_.highlight_intensity_threshold) {
        state = Image_State::BACKLIGHT;
      }
      states.push_back(state);
    }
    return states;
  }

  void ImageDiagNode::update_image_diagnostics(const std::vector<Image_State> & states)
  {
    diagnostics_interface_->clear();

    const auto total_blocks = params_.num_blocks_horizontal * params_.num_blocks_vertical;
    const auto num_normal = std::count(states.begin(), states.end(), Image_State::NORMAL);
    const auto num_shadow = std::count(states.begin(), states.end(), Image_State::DARK);
    const auto num_blockage = std::count(states.begin(), states.end(), Image_State::BLOCKAGE);
    const auto num_low_vis = std::count(states.begin(), states.end(), Image_State::LOW_VIS);
    const auto num_highlight = std::count(states.begin(), states.end(), Image_State::BACKLIGHT);

    const auto ratio_normal = static_cast<float>(num_normal) / static_cast<float>(total_blocks);
    const auto ratio_blockage = static_cast<float>(num_blockage) / static_cast<float>(total_blocks);
    const auto ratio_shadow = static_cast<float>(num_shadow) / static_cast<float>(total_blocks);
    const auto ratio_low_vis = static_cast<float>(num_low_vis) / static_cast<float>(total_blocks);
    const auto ratio_highlight =
      static_cast<float>(num_highlight) / static_cast<float>(total_blocks);

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> status_details;

    auto check_status = [&](
                          const std::string & label, int64_t count, int warn_thresh, int err_thresh,
                          const std::string & key_prefix, float ratio) {
      std::string status = "OK";
      if (count > err_thresh) {
        status = "ERROR";
        level = std::max(level, static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
        status_details.emplace_back(
          label + ": ERROR (count = " + std::to_string(count) +
          ", error threshold = " + std::to_string(err_thresh) + ")");
      } else if (count > warn_thresh) {
        status = "WARNING";
        level = std::max(level, static_cast<int8_t>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
        status_details.emplace_back(
          label + ": WARNING (count = " + std::to_string(count) +
          ", warning threshold = " + std::to_string(warn_thresh) + ")");
      }
      diagnostics_interface_->add_key_value(key_prefix + "_status", status);
      diagnostics_interface_->add_key_value(key_prefix + "_number", std::to_string(count));
      diagnostics_interface_->add_key_value(key_prefix + "_ratio", std::to_string(ratio));
    };

    diagnostics_interface_->add_key_value("normal_ratio", std::to_string(ratio_normal));

    check_status(
      "Blockage", num_blockage, params_.blockage_region_warn_threshold,
      params_.blockage_region_error_threshold, "blockage", ratio_blockage);

    check_status(
      "Highlight clipping", num_highlight, params_.highlight_region_warn_threshold,
      params_.highlight_region_error_threshold, "highlight_clipping", ratio_highlight);

    check_status(
      "Shadow clipping", num_shadow, params_.shadow_region_warn_threshold,
      params_.shadow_region_error_threshold, "shadow_clipping", ratio_shadow);

    check_status(
      "Low visibility", num_low_vis, params_.low_visibility_region_warn_threshold,
      params_.low_visibility_region_error_threshold, "low_visibility", ratio_low_vis);

    std::ostringstream status_msg;
    if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      status_msg << "Image status: "
                 << (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ? "ERROR" : "WARNING");
      status_msg << "\nDetails:\n";
      for (const auto & msg : status_details) {
        status_msg << "- " << msg << "\n";
      }
      region_state_vec.push_back(region_state);
    }

    diagnostics_interface_->update_level_and_message(level, status_msg.str());
    diagnostics_interface_->publish(this->get_clock()->now());
  }

  cv::Mat ImageDiagNode::draw_diagnostic_overlay(
    const std::vector<Image_State> & states, const cv::Size & size)
  {
    cv::Mat diag_block_image(size, CV_8UC3);
    int block_w = std::floor(size.width / params_.num_blocks_horizontal);
    int block_h = std::floor(size.height / params_.num_blocks_vertical);

    int idx = 0;
    for (int v = 0; v < params_.num_blocks_vertical; ++v) {
      for (int h = 0; h < params_.num_blocks_horizontal; ++h, ++idx) {
        cv::Rect r(h * block_w, v * block_h, block_w, block_h);
        cv::rectangle(diag_block_image, r, state_color_map_.at(states[idx]), -1);
      }
    }
    // draw boundary of blocks
    for (int v = 1; v < params_.number_block_vertical; v++) {
      cv::line(
        diag_block_image, cv::Point(0, v * block_h), cv::Point(size.width, v * block_h),
        border_color_, 1, cv::LINE_AA, 0);
      for (int h = 1; h < params_.num_blocks_horizontal; ++h)
        cv::line(
          diag_block_image, cv::Point(h * block_w, 0), cv::Point(h * block_w, size.height),
          border_color_, 1, cv::LINE_AA, 0);
      return diag_block_image;
    }

    gray_image_pub_.publish(gray_image_msg);
    dft_image_pub_.publish(dft_image_msg);
    block_diag_image_pub_.publish(block_diag_image_msg);
  }

  void ImageDiagNode::shift_image(cv::Mat & img)
  {
    int cx = img.cols / 2;
    int cy = img.rows / 2;

    cv::Mat left_top(img, cv::Rect(0, 0, cx, cy));
    cv::Mat right_top(img, cv::Rect(cx, 0, cx, cy));
    cv::Mat left_bottom(img, cv::Rect(0, cy, cx, cy));
    cv::Mat right_bottom(img, cv::Rect(cx, cy, cx, cy));

    cv::Mat tmp;

    left_top.copyTo(tmp);
    right_bottom.copyTo(left_top);
    tmp.copyTo(right_bottom);

    right_top.copyTo(tmp);
    left_bottom.copyTo(right_top);
    tmp.copyTo(left_bottom);
  }
}  // namespace autoware::image_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_diagnostics::ImageDiagNode)
