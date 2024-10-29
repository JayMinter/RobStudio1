# cmake_minimum_required(VERSION 3.8)
# project(sprint4)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)

# # Add executables
# add_executable(cylinder_node src/cylinder.cpp)
# add_executable(main_node src/main.cpp src/cylinder.cpp)

# # Link dependencies for each executable
# ament_target_dependencies(cylinder_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3)
# ament_target_dependencies(main_node rclcpp sensor_msgs geometry_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3)

# # Install executables
# install(TARGETS
#   cylinder_node
#   main_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files
# install(
#   DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Export package
# ament_package()

cmake_minimum_required(VERSION 3.8)
project(sprint4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)

# Add single main executable
add_executable(mainrun src/main.cpp src/cylinder.cpp)

# Link dependencies
ament_target_dependencies(mainrun rclcpp cv_bridge sensor_msgs geometry_msgs nav_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs Eigen3)

# Install main executable
install(TARGETS
  mainrun
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export package
ament_package()
