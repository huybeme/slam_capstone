set(_AMENT_PACKAGE_NAME "laser_geometry")
set(laser_geometry_VERSION "2.4.0")
set(laser_geometry_MAINTAINER "Mabel Zhang <mabel@openrobotics.org>")
set(laser_geometry_BUILD_DEPENDS "eigen" "rclcpp" "sensor_msgs" "tf2")
set(laser_geometry_BUILDTOOL_DEPENDS "ament_cmake" "eigen3_cmake_module")
set(laser_geometry_BUILD_EXPORT_DEPENDS "eigen")
set(laser_geometry_BUILDTOOL_EXPORT_DEPENDS "eigen3_cmake_module")
set(laser_geometry_EXEC_DEPENDS "python3-numpy" "rclcpp" "rclpy" "sensor_msgs" "sensor_msgs_py" "tf2")
set(laser_geometry_TEST_DEPENDS "ament_cmake_cppcheck" "ament_cmake_cpplint" "ament_cmake_gtest" "ament_cmake_lint_cmake" "ament_cmake_pytest" "ament_cmake_uncrustify" "python_cmake_module")
set(laser_geometry_GROUP_DEPENDS )
set(laser_geometry_MEMBER_OF_GROUPS )
set(laser_geometry_DEPRECATED "")
set(laser_geometry_EXPORT_TAGS)
list(APPEND laser_geometry_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
