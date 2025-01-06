cmake_minimum_required(VERSION 3.8)
project(robot_arm_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

include(CMakePackageConfigHelpers)

add_subdirectory(src)

pluginlib_export_plugin_description_file(hardware_interface robot_arm_hw.xml)

ament_package()
