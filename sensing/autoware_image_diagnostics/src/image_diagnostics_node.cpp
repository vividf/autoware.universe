// Copyright 2025 TIER IV, Inc.
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

#include "autoware/image_diagnostics/image_diagnostics_node.hpp"

#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::image_diagnostics
{

ImageDiagNode::ImageDiagNode(const rclcpp::NodeOptions & node_options)
: Node("image_diagnostics_node", node_options)
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "input/raw_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageDiagNode::run_image_diagnostics, this, std::placeholders::_1));
  block_diag_image_pub_ =
    image_transport::create_publisher(this, "image_diag/debug/diag_block_image");
  dft_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/dft_image");
  gray_image_pub_ = image_transport::create_publisher(this, "image_diag/debug/gray_image");

  image_state_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>(
    "image_diag/image_state_diag", rclcpp::SensorDataQoS());

  // Parameters
  // Parameters
  params_.image_resize_height = declare_parameter<int>("image_resize_height");
  params_.num_blocks_horizontal = declare_parameter<int>("num_blocks_horizontal");
  params_.num_blocks_vertical = declare_parameter<int>("num_blocks_vertical");

  // Blockage thresholds
  params_.blockage_region_warn_threshold = declare_parameter<int>("blockage_region_warn_threshold");
  params_.blockage_region_error_threshold =
    declare_parameter<int>("blockage_region_error_threshold");
  params_.blockage_ratio_threshold = declare_parameter<float>("blockage_ratio_threshold");
  params_.blockage_intensity_threshold = declare_parameter<int>("blockage_intensity_threshold");
  params_.blockage_frequency_ratio_threshold =
    declare_parameter<float>("blockage_frequency_ratio_threshold");

  // Shadow clipping thresholds
  params_.shadow_region_warn_threshold = declare_parameter<int>("shadow_region_warn_threshold");
  params_.shadow_region_error_threshold = declare_parameter<int>("shadow_region_error_threshold");
  params_.shadow_intensity_threshold = declare_parameter<int>("shadow_intensity_threshold");

  // Highlight clipping thresholds
  params_.highlight_region_warn_threshold =
    declare_parameter<int>("highlight_region_warn_threshold");
  params_.highlight_region_error_threshold =
    declare_parameter<int>("highlight_region_error_threshold");
  params_.highlight_intensity_threshold = declare_parameter<int>("highlight_intensity_threshold");

  // Low visibility thresholds
  params_.low_visibility_region_warn_threshold =
    declare_parameter<int>("low_visibility_region_warn_threshold");
  params_.low_visibility_region_error_threshold =
    declare_parameter<int>("low_visibility_region_error_threshold");
  params_.low_visibility_frequency_threshold =
    declare_parameter<float>("low_visibility_frequency_threshold");
  diagnostics_interface_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, this->get_fully_qualified_name());
}

void ImageDiagNode::run_image_diagnostics(
  const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg)
{
  const cv::Mat gray_img = preprocess_image(input_image_msg);
  const RegionFeatures features = compute_image_features(gray_img);
  const std::vector<int> region_states = classify_regions(features);

  const cv::Mat diag_block_image = draw_diagnostic_overlay(region_states, gray_img.size());
  publish_debug_images(input_image_msg->header, gray_img, features.freq_map, diag_block_image);
  update_image_diagnostics(region_states);
}

cv::Mat ImageDiagNode::preprocess_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  cv::Mat gray_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
  cv::Size new_size(
    params_.image_resize_height * gray_img.cols / gray_img.rows, params_.image_resize_height);
  cv::resize(gray_img, gray_img, new_size);
  return gray_img;
}

ImageDiagNode::RegionFeatures ImageDiagNode::compute_image_features(
  const cv::Mat & gray_image) const
{
  RegionFeatures features;

  // Compute the width and height of each block
  const int block_w = std::floor(gray_image.cols / params_.num_blocks_horizontal);
  const int block_h = std::floor(gray_image.rows / params_.num_blocks_vertical);
  const int region_pix_count = block_w * block_h;
  // Get optimal sizes for DFT computation (padded sizes)
  const int dft_w = cv::getOptimalDFTSize(block_w);
  const int dft_h = cv::getOptimalDFTSize(block_h);

  // Threshold image to find low-intensity (possibly blocked) pixels
  cv::Mat bin_image;
  cv::threshold(
    gray_image, bin_image, params_.blockage_intensity_threshold, 255, cv::THRESH_BINARY);

  // Convert to 32-bit float for frequency domain processing
  cv::Mat gray_image_32f;
  gray_image.convertTo(gray_image_32f, CV_32FC1);
  features.freq_map = cv::Mat(gray_image.size(), CV_32FC1, cv::Scalar(0));

  // Iterate over each image block
  for (int v = 0; v < params_.num_blocks_vertical; ++v) {
    for (int h = 0; h < params_.num_blocks_horizontal; ++h) {
      cv::Rect roi(h * block_w, v * block_h, block_w, block_h);

      // Compute average intensity and blockage ratio
      int avg_intensity = static_cast<int>(cv::mean(gray_image(roi))[0]);
      int blocked = cv::countNonZero(bin_image(roi));
      float ratio = static_cast<float>(region_pix_count - blocked) / region_pix_count;

      // Extract and pad block for DFT
      cv::Mat gray_img_roi = gray_image_32f(roi);
      cv::Mat padded_roi;
      cv::copyMakeBorder(
        gray_img_roi, padded_roi, 0, dft_h - block_h, 0, dft_w - block_w, cv::BORDER_CONSTANT);

      // Perform DFT to obtain frequency components
      std::vector<cv::Mat> channel_img = {padded_roi, cv::Mat::zeros(padded_roi.size(), CV_32FC1)};
      cv::Mat complex_img;
      cv::merge(channel_img, complex_img);
      cv::dft(complex_img, complex_img);
      shift_image(complex_img);
      cv::split(complex_img, channel_img);

      // Crop back to original block size and compute log frequency spectrum
      cv::Mat freq_roi = channel_img[0](cv::Rect(0, 0, block_w, block_h));
      cv::log(freq_roi, freq_roi);
      float freq_mean = static_cast<float>(cv::mean(freq_roi)[0]);

      // Store features
      freq_roi.copyTo(features.freq_map(roi));
      features.avg_intensity.push_back(avg_intensity);
      features.blockage_ratio.push_back(ratio);
      features.freq_sum.push_back(freq_mean);
    }
  }
  return features;
}

std::vector<int> ImageDiagNode::classify_regions(const RegionFeatures & features) const
{
  std::vector<int> states;
  for (size_t i = 0; i < features.avg_intensity.size(); ++i) {
    int state = Image_State::NORMAL;
    if (features.avg_intensity[i] < params_.shadow_intensity_threshold) {
      state = Image_State::DARK;
    } else if (
      features.blockage_ratio[i] > params_.blockage_ratio_threshold &&
      features.freq_sum[i] < params_.blockage_frequency_ratio_threshold) {
      state = Image_State::BLOCKAGE;
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

void ImageDiagNode::update_image_diagnostics(const std::vector<int> & states)
{
  diagnostics_interface_->clear();

  auto num_of_normal_region = std::count(states.begin(), states.end(), Image_State::NORMAL);
  auto num_of_shadow_region = std::count(states.begin(), states.end(), Image_State::DARK);
  auto num_of_blockage_region = std::count(states.begin(), states.end(), Image_State::BLOCKAGE);
  auto num_of_low_visibility_region =
    std::count(states.begin(), states.end(), Image_State::LOW_VIS);
  auto num_of_highligt_region = std::count(states.begin(), states.end(), Image_State::BACKLIGHT);

  const int total_blocks = params_.num_blocks_horizontal * params_.num_blocks_vertical;
  const float normal_ratio =
    static_cast<float>(num_of_normal_region) / static_cast<float>(total_blocks);
  const float blockage_ratio =
    static_cast<float>(num_of_blockage_region) / static_cast<float>(total_blocks);
  const float highlight_ratio =
    static_cast<float>(num_of_highligt_region) / static_cast<float>(total_blocks);
  const float shadow_ratio =
    static_cast<float>(num_of_shadow_region) / static_cast<float>(total_blocks);
  const float low_vis_ratio =
    static_cast<float>(num_of_low_visibility_region) / static_cast<float>(total_blocks);

  std::vector<std::string> error_messages;

  // Normal
  diagnostics_interface_->add_key_value("normal_ratio", std::to_string(normal_ratio));

  // Blockage
  std::string blockage_status = "OK";
  if (num_of_blockage_region > params_.blockage_region_error_threshold) {
    blockage_status = "ERROR";
    error_messages.emplace_back("Blockage status: ERROR (above threshold)");
  } else if (num_of_blockage_region > params_.blockage_region_warn_threshold) {
    blockage_status = "WARN";
  }
  diagnostics_interface_->add_key_value("blockage_status", blockage_status);
  diagnostics_interface_->add_key_value("blockage_ratio", std::to_string(blockage_ratio));

  // Highlight clipping
  std::string highlight_status = "OK";
  if (num_of_highligt_region > params_.highlight_region_error_threshold) {
    highlight_status = "ERROR";
    error_messages.emplace_back("Highlight clipping status: ERROR (above threshold)");
  } else if (num_of_highligt_region > params_.highlight_region_warn_threshold) {
    highlight_status = "WARN";
  }
  diagnostics_interface_->add_key_value("highlight_clipping_status", highlight_status);
  diagnostics_interface_->add_key_value("highlight_ratio", std::to_string(highlight_ratio));

  // Shadow clipping
  std::string shadow_status = "OK";
  if (num_of_shadow_region > params_.shadow_region_error_threshold) {
    shadow_status = "ERROR";
    error_messages.emplace_back("Shadow clipping status: ERROR (above threshold)");
  } else if (num_of_shadow_region > params_.shadow_region_warn_threshold) {
    shadow_status = "WARN";
  }
  diagnostics_interface_->add_key_value("shadow_clipping_status", shadow_status);
  diagnostics_interface_->add_key_value("shadow_ratio", std::to_string(shadow_ratio));

  // Low visibility
  std::string low_vis_status = "OK";
  if (num_of_low_visibility_region > params_.low_visibility_region_error_threshold) {
    low_vis_status = "ERROR";
    error_messages.emplace_back("Low visibility status: ERROR (above threshold)");
  } else if (num_of_low_visibility_region > params_.low_visibility_region_warn_threshold) {
    low_vis_status = "WARN";
  }
  diagnostics_interface_->add_key_value("low_visibility_status", low_vis_status);
  diagnostics_interface_->add_key_value("low_visibility_ratio", std::to_string(low_vis_ratio));

  // Summary diagnostic message

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (
    blockage_status == "ERROR" || highlight_status == "ERROR" || shadow_status == "ERROR" ||
    low_vis_status == "ERROR") {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (
    blockage_status == "WARN" || highlight_status == "WARN" || shadow_status == "WARN" ||
    low_vis_status == "WARN") {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  std::ostringstream status_msg;

  if (!error_messages.empty()) {
    status_msg << "\nDetails:\n";
    if (blockage_status != "OK") {
      status_msg << "- Blockage: " << blockage_status << " (value = " << blockage_ratio
                 << ", warn threshold = "
                 << static_cast<float>(params_.blockage_region_warn_threshold) /
                      static_cast<float>(total_blocks)
                 << ", error threshold = "
                 << static_cast<float>(params_.blockage_region_error_threshold) /
                      static_cast<float>(total_blocks)
                 << ")\n";
    }
    if (highlight_status != "OK") {
      status_msg << "- Highlight clipping: " << highlight_status << " (value = " << highlight_ratio
                 << ", warn threshold = "
                 << static_cast<float>(params_.highlight_region_warn_threshold) /
                      static_cast<float>(total_blocks)
                 << ", error threshold = "
                 << static_cast<float>(params_.highlight_region_error_threshold) /
                      static_cast<float>(total_blocks)
                 << ")\n";
    }
    if (shadow_status != "OK") {
      status_msg << "- Shadow clipping: " << shadow_status << " (value = " << shadow_ratio
                 << ", warn threshold = "
                 << static_cast<float>(params_.shadow_region_warn_threshold) /
                      static_cast<float>(total_blocks)
                 << ", error threshold = "
                 << static_cast<float>(params_.shadow_region_error_threshold) /
                      static_cast<float>(total_blocks)
                 << ")\n";
    }
    if (low_vis_status != "OK") {
      status_msg << "- Low visibility: " << low_vis_status << " (value = " << low_vis_ratio
                 << ", warn threshold = "
                 << static_cast<float>(params_.low_visibility_region_warn_threshold) /
                      static_cast<float>(total_blocks)
                 << ", error threshold = "
                 << static_cast<float>(params_.low_visibility_region_error_threshold) /
                      static_cast<float>(total_blocks)
                 << ")\n";
    }
  }

  diagnostics_interface_->update_level_and_message(level, status_msg.str());
  diagnostics_interface_->publish(this->get_clock()->now());
}

cv::Mat ImageDiagNode::draw_diagnostic_overlay(
  const std::vector<int> & states, const cv::Size & size)
{
  cv::Mat diag_block_image(size, CV_8UC3);
  int block_w = std::floor(size.width / params_.num_blocks_horizontal);
  int block_h = std::floor(size.height / params_.num_blocks_vertical);

  int idx = 0;
  for (int v = 0; v < params_.num_blocks_vertical; ++v) {
    for (int h = 0; h < params_.num_blocks_horizontal; ++h, ++idx) {
      cv::Rect r(h * block_w, v * block_h, block_w, block_h);
      const std::string state = get_state_string(states[idx]);
      cv::rectangle(diag_block_image, r, state_color_map_.at(state), -1);
    }
  }

  // draw boundary of blocks
  for (int v = 1; v < params_.num_blocks_vertical; ++v)
    cv::line(
      diag_block_image, cv::Point(0, v * block_h), cv::Point(size.width, v * block_h),
      state_color_map_["BORDER"], 1, cv::LINE_AA, 0);
  for (int h = 1; h < params_.num_blocks_horizontal; ++h)
    cv::line(
      diag_block_image, cv::Point(h * block_w, 0), cv::Point(h * block_w, size.height),
      state_color_map_["BORDER"], 1, cv::LINE_AA, 0);
  return diag_block_image;
}

void ImageDiagNode::publish_debug_images(
  const std_msgs::msg::Header & header, const cv::Mat & gray_image, const cv::Mat & dft_image,
  const cv::Mat & diag_block_image)
{
  auto gray_image_msg = cv_bridge::CvImage(header, "mono8", gray_image).toImageMsg();
  auto dft_image_msg = cv_bridge::CvImage(header, "mono8", dft_image).toImageMsg();
  auto block_diag_image_msg = cv_bridge::CvImage(header, "bgr8", diag_block_image).toImageMsg();
  gray_image_pub_.publish(gray_image_msg);
  dft_image_pub_.publish(dft_image_msg);
  block_diag_image_pub_.publish(block_diag_image_msg);
}

std::string ImageDiagNode::get_state_string(int state)
{
  switch (state) {
    case Image_State::DARK:
      return "DARK";
    case Image_State::BLOCKAGE:
      return "BLOCKAGE";
    case Image_State::LOW_VIS:
    case Image_State::BACKLIGHT:
      return "BACKLIGHT";
    case Image_State::NORMAL:
    default:
      return "NORMAL";
  }
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
