cmake_minimum_required(VERSION 3.5)
project(capstone_pkg_cpp)

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
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(odom_map_transform src/odom_map_transform.cpp)
ament_target_dependencies(odom_map_transform rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS
  odom_map_transform
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
