cmake_minimum_required(VERSION 3.8)
project(image_overlay)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(image_overlay_node src/image_overlay.cpp)

ament_target_dependencies(image_overlay_node rclcpp OpenCV)

# Install the executable
install(TARGETS
  image_overlay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

