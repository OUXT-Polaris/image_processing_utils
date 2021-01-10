// Copyright (c) 2020 OUXT Polaris
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

#include <image_processing_utils/image_rectify_component.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>

namespace image_processing_utils
{
ImageRectifyComponent::ImageRectifyComponent(const rclcpp::NodeOptions & options)
: Node("image_rectify_node", options)
{
  declare_parameter("interpolation", 1);
  get_parameter("interpolation", interpolation_);
  pub_rect_ = image_transport::create_publisher(this, "image_rect");
  sub_camera_ = image_transport::create_camera_subscription(
    this, "image", std::bind(
      &ImageRectifyComponent::callback,
      this, std::placeholders::_1, std::placeholders::_2), "raw");
}

void ImageRectifyComponent::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
  if (pub_rect_.getNumSubscribers() < 1) {
    return;
  }
  model_.fromCameraInfo(*camera_info);
  const cv::Mat image_cv = cv_bridge::toCvShare(image)->image;
  cv::Mat rect_image_cv;
  model_.rectifyImage(image_cv, rect_image_cv, interpolation_);
  sensor_msgs::msg::Image::SharedPtr rect_image =
    cv_bridge::CvImage(image->header, image->encoding, rect_image_cv).toImageMsg();
  pub_rect_.publish(rect_image);
}
}  // namespace image_processing_utils

RCLCPP_COMPONENTS_REGISTER_NODE(image_processing_utils::ImageRectifyComponent)
