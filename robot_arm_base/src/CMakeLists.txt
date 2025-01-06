add_library(robot_arm_base SHARED robot_arm_interface.cpp)

target_include_directories(robot_arm_base PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/robot_arm_base>
)

ament_target_dependencies(robot_arm_base PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(
  TARGETS robot_arm_base
  EXPORT robot_arm_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(robot_arm_base HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)