cmake_minimum_required(VERSION 3.5)
project(arctos_hardware_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Add library for STM32 Stepper hardware interface (legacy v1)
add_library(arctos_stm32_stepper_interface SHARED
  src/stm32_stepper_interface.cpp
  src/utils/unit_conversion.cpp
  src/utils/stm32_protocol.cpp
  src/utils/socket_manager.cpp
)

target_link_libraries(arctos_stm32_stepper_interface
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES} 
)

target_include_directories(arctos_stm32_stepper_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arctos_stm32_stepper_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Add library for STM32 Trajectory hardware interface (v2)
add_library(arctos_stm32_trajectory_interface SHARED
  src/stm32_trajectory_interface.cpp
  src/utils/unit_conversion.cpp
  src/utils/stm32_protocol_v2.cpp
  src/utils/socket_manager.cpp
)

target_link_libraries(arctos_stm32_trajectory_interface
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES}
)

target_include_directories(arctos_stm32_trajectory_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arctos_stm32_trajectory_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Export the plugin description file
pluginlib_export_plugin_description_file(hardware_interface "arctos_hardware_interface.xml")

# Install targets
install(TARGETS 
  arctos_stm32_stepper_interface
  arctos_stm32_trajectory_interface
  EXPORT export_arctos_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

install(FILES arctos_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_targets(export_arctos_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  hardware_interface 
  pluginlib
  rclcpp_lifecycle
)

ament_package()
