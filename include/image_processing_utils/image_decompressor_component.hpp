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

#ifndef IMAGE_PROCESSING_UTILS__IMAGE_DECOMPRESSOR_COMPONENT_HPP_
#define IMAGE_PROCESSING_UTILS__IMAGE_DECOMPRESSOR_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_EXPORT __attribute__((dllexport))
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_EXPORT __declspec(dllexport)
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_BUILDING_DLL
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC \
  IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_EXPORT
#else
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC \
  IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_IMPORT
#endif
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC_TYPE \
  IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_LOCAL
#else
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_LOCAL
#endif
#define IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace image_processing_utils
{
class ImageDecompressorComponent : public rclcpp::Node
{
public:
  IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_PUBLIC
  explicit ImageDecompressorComponent(const rclcpp::NodeOptions & options);

private:
  void ImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  struct Config
  {
    int imdecode_flag;
  };
  Config config_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};
}  // namespace image_processing_utils

#endif  // IMAGE_PROCESSING_UTILS__IMAGE_DECOMPRESSOR_COMPONENT_HPP_
