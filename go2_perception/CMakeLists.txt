cmake_minimum_required(VERSION 3.18)          # 3.18+ supports LINK_OPTIONS
project(go2_perception)

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
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pybind11 REQUIRED)

# Disable RPATH manipulation to avoid install_name_tool errors
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

include_directories(
  include
)

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

target_include_directories(pointcloud_to_laserscan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")

target_link_libraries(pointcloud_to_laserscan PUBLIC
  pybind11::embed                      # libpython integration
  Python3::Python
  laser_geometry::laser_geometry
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)



rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

# Fix Python linking for the executable
target_link_libraries(pointcloud_to_laserscan_node Python3::Python)

# Set RPATH for the executable to find libpython
if(DEFINED ENV{CONDA_PREFIX})
  set_target_properties(pointcloud_to_laserscan_node PROPERTIES
    INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib"
    BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
endif()

install(TARGETS
  pointcloud_to_laserscan
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

add_executable(cloud_accumulation
  src/cloud_accumulation.cpp
)
target_link_libraries(cloud_accumulation PUBLIC
  Python3::Python
  laser_geometry::laser_geometry
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)

# Fix Python linking and runtime path for cloud_accumulation
if(DEFINED ENV{CONDA_PREFIX})
  set_target_properties(cloud_accumulation PROPERTIES
    INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib"
    BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
endif()

install(TARGETS
  cloud_accumulation
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Export dependencies including Python
ament_export_dependencies(rclcpp Python3 laser_geometry message_filters sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

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

ament_package()
