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

#include <memory>

namespace image_processing_utils
{
ImageRectifyComponent::ImageRectifyComponent(const rclcpp::NodeOptions & options)
: Node("image_rectify_node", options)
{
  camera_info_sub_ =
    std::shared_ptr<CameraInfoSubscriber>(new CameraInfoSubscriber(this, "camera_info"));
  image_sub_ = std::shared_ptr<ImageSubscriber>(new ImageSubscriber(this, "image_raw"));
  sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::Image>>(*camera_info_sub_,
      *image_sub_, 10);
  sync_->registerCallback(std::bind(
      &ImageRectifyComponent::callback, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ImageRectifyComponent::callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info,
  const sensor_msgs::msg::Image::ConstSharedPtr image)
{
}
}  // namespace image_processing_utils
