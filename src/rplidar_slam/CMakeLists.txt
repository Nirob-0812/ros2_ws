cmake_minimum_required(VERSION 3.8)
project(agv_proto)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required ROS 2 and other packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros_odrive_msg REQUIRED)
find_package(class_loader REQUIRED)
find_package(console_bridge_vendor REQUIRED)

# System packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Auto-discover source files
file(GLOB_RECURSE AGV_PROTO_SOURCES
  "src/*.cpp"
)

# Remove files with conflicting main functions for now
list(REMOVE_ITEM AGV_PROTO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/my_nav2_goal_client.cpp")

# Create executable
add_executable(agv_node ${AGV_PROTO_SOURCES})

# Include headers
target_include_directories(agv_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBUSB_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

# Link ROS 2 and system libraries
ament_target_dependencies(agv_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  ros_odrive_msg
  class_loader
  console_bridge_vendor
)

target_link_libraries(agv_node
  ${LIBUSB_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

# Required for class_loader plugins
class_loader_hide_library_symbols(agv_node)

# Install the node binary
install(TARGETS agv_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

# Export all required dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  ros_odrive_msg
  class_loader
  console_bridge_vendor
)

ament_package()
