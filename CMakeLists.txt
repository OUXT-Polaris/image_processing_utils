cmake_minimum_required(VERSION 3.5)
project(image_processing_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(image_decompressor_component SHARED
  src/image_decompressor_component.cpp)
target_compile_definitions(image_decompressor_component
  PRIVATE "IMAGE_PROCESSING_UTILS_IMAGE_DECOMPRESSOR_COMPONENT_BUILDING_DLL")
ament_target_dependencies(image_decompressor_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge)
rclcpp_components_register_nodes(image_decompressor_component
  "image_processing_utils::ImageDecompressorComponent")
ament_export_libraries(image_decompressor_component)

add_executable(image_decompressor_node
  src/image_decompressor_node.cpp)
ament_target_dependencies(image_decompressor_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge)
target_link_libraries(image_decompressor_node image_decompressor_component)

add_library(image_rectify_component SHARED
  src/image_rectify_component.cpp)
target_compile_definitions(image_rectify_component
  PRIVATE "IMAGE_PROCESSING_UTILS_IMAGE_RECTIFY_COMPONENT_BUILDING_DLL")
ament_target_dependencies(image_rectify_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry)
rclcpp_components_register_nodes(image_rectify_component
  "image_processing_utils::ImageRectifyComponent")
ament_export_libraries(image_rectify_component)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS
  image_decompressor_node
  DESTINATION lib/image_processing_utils
)
install(TARGETS image_decompressor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
