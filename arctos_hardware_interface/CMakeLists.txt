cmake_minimum_required(VERSION 3.5)
project(arctos_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(arctos_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(arctos_hardware_interface SHARED
  src/stm32_hardware_interface.cpp
)

target_include_directories(arctos_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arctos_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  arctos_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface "arctos_hardware_interface.xml")

install(TARGETS arctos_hardware_interface
  EXPORT export_arctos_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES arctos_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_arctos_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  arctos_msgs
  std_msgs
)

ament_package()
