cmake_minimum_required(VERSION 3.8)
project(my_bot_imu)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(my_bot_imu src/my_bot_imu.cpp)

# target_link_libraries(my_bot_imu
#   tf2
#   tf2_ros
# )


# Include directories
target_include_directories(my_bot_imu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Specify dependencies of the executable
ament_target_dependencies(my_bot_imu
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
# ament_target_dependencies(bno055_subscriber
#   rclcpp
#   sensor_msgs
#   tf2
#   tf2_geometry_msgs
# )
# If using custom messages, link against typesupport
# rosidl_target_interfaces(my_bot_imu
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# Install the executable
install(TARGETS 
  my_bot_imu
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# # Install RViz config files
# install(
#   DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}/
# )



ament_package()