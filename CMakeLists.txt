cmake_minimum_required(VERSION 3.8)
project(lidar_rgb_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # We require C++ 17

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(PCL REQUIRED COMPONENTS)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen REQUIRED)

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

include_directories(
  ${cv_bridge_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  include
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(CloudFusion src/fusion.cpp)

ament_target_dependencies(CloudFusion
  cv_bridge
  rclcpp
  sensor_msgs
  PCL
  OpenCV
  tf2
  tf2_ros
  pcl_conversions
  Eigen
)

target_link_libraries(CloudFusion
  ${PCL_LIBRARIES}
)

install(
 TARGETS CloudFusion
 DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()