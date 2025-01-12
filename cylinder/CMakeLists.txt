cmake_minimum_required(VERSION 3.8)
project(cylinder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(cylinder_node src/cylinder.cpp)

ament_target_dependencies(cylinder_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)

# Install executable
install(TARGETS
  cylinder_node
  DESTINATION lib/${PROJECT_NAME})

# Export package
ament_package()
