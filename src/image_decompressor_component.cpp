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

#include <image_processing_utils/image_decompressor_component.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

namespace image_processing_utils
{
ImageDecompressorComponent::ImageDecompressorComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_decompressor", options)
{
  std::string in_topic = rclcpp::expand_topic_or_service_name("in", get_name(), get_namespace());
  std::string out_topic = rclcpp::expand_topic_or_service_name("out", get_name(), get_namespace());
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 1);
  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    in_topic, 1,
    std::bind(&ImageDecompressorComponent::ImageCallback, this, std::placeholders::_1));
}

void ImageDecompressorComponent::ImageCallback(
  const sensor_msgs::msg::CompressedImage::SharedPtr message)
{
  namespace enc = sensor_msgs::image_encodings;
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

  // Copy message header
  cv_ptr->header = message->header;

  // Decode color/mono image
  try {
    cv_ptr->image = cv::imdecode(cv::Mat(message->data), config_.imdecode_flag);

    // Assign image encoding string
    const size_t split_pos = message->format.find(';');
    if (split_pos == std::string::npos) {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels()) {
        case 1:
          cv_ptr->encoding = enc::MONO8;
          break;
        case 3:
          cv_ptr->encoding = enc::BGR8;
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Unsupported number of channels: %i",
            cv_ptr->image.channels());
          break;
      }
    } else {
      std::string image_encoding = message->format.substr(0, split_pos);

      cv_ptr->encoding = image_encoding;

      if (enc::isColor(image_encoding)) {
        std::string compressed_encoding = message->format.substr(split_pos);
        bool compressed_bgr_image =
          (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if ((image_encoding == enc::RGB8) || (image_encoding == enc::RGB16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if ((image_encoding == enc::BGR8) || (image_encoding == enc::BGR16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
          }

          if ((image_encoding == enc::BGRA8) || (image_encoding == enc::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
          }

          if ((image_encoding == enc::RGBA8) || (image_encoding == enc::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
          }
        }
      }
    }
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;
  if ((rows > 0) && (cols > 0)) {
    image_pub_->publish(*cv_ptr->toImageMsg());
  }
}
}  // namespace image_processing_utils

RCLCPP_COMPONENTS_REGISTER_NODE(image_processing_utils::ImageDecompressorComponent)
