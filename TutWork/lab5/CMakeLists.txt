cmake_minimum_required(VERSION 3.8)
project(lab5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(scan_to_image src/scan_to_image_node.cpp)
ament_target_dependencies(scan_to_image rclcpp sensor_msgs OpenCV)


install(TARGETS
scan_to_image
DESTINATION lib/${PROJECT_NAME})

ament_package()
