cmake_minimum_required(VERSION 3.8)
project(lab4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(deadreckoning src/deadreckoning.cpp)
ament_target_dependencies(deadreckoning 
rclcpp 
nav_msgs
tf2 
tf2_ros 
tf2_geometry_msgs
message_filters 
)


install(TARGETS
deadreckoning
DESTINATION lib/${PROJECT_NAME})

ament_package()

