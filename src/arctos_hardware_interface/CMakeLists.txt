cmake_minimum_required(VERSION 3.5)
project(arctos_hardware_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arctos_motor_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)







# Add library for new Arctos MKS hardware interface
add_library(arctos_mks_hardware_interface SHARED
  src/arctos_mks_hardware_interface.cpp
)

# Link the Arctos MKS hardware interface libraries and dependencies
target_link_libraries(arctos_mks_hardware_interface
  ${rclcpp_LIBRARIES}
  ${hardware_interface_LIBRARIES} 
)

# Include directories
target_include_directories(arctos_mks_hardware_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arctos_mks_hardware_interface
  rclcpp
  can_msgs
  sensor_msgs
  arctos_motor_driver  
  hardware_interface
  pluginlib
)

# Export the plugin description file
pluginlib_export_plugin_description_file(hardware_interface "arctos_hardware_interface.xml")

# Install targets
install(TARGETS 
  arctos_mks_hardware_interface
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

# install(DIRECTORY config 
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY ros2_control
#   DESTINATION share/${PROJECT_NAME}/ros2_control
# )


install(FILES arctos_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_targets(export_arctos_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  can_msgs 
  sensor_msgs 
  hardware_interface 
  std_srvs
  std_msgs
)

ament_package()