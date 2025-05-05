cmake_minimum_required(VERSION 3.8)
project(custom_hardware_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  custom_hardware_plugins SHARED
  src/velocity_integrator_hardware.cpp
)

target_include_directories(custom_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_hardware_plugins PUBLIC
  hardware_interface
  pluginlib
  rclcpp
)
# Export the plugin description file
pluginlib_export_plugin_description_file(custom_hardware_plugins plugins.xml)

install(
  TARGETS custom_hardware_plugins
  EXPORT custom_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_export_targets(custom_hardware_plugins HAS_LIBRARY_TARGET)

ament_package()