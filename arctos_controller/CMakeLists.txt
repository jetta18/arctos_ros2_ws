cmake_minimum_required(VERSION 3.8)
project(arctos_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(arctos_hardware_interface REQUIRED)

include_directories(include)

add_library(arctos_segment_controller SHARED
  src/arctos_segment_controller.cpp
)
target_compile_features(arctos_segment_controller PUBLIC cxx_std_17)
ament_target_dependencies(arctos_segment_controller
  controller_interface
  rclcpp
  rclcpp_action
  pluginlib
  control_msgs
  trajectory_msgs
  arctos_hardware_interface
)

pluginlib_export_plugin_description_file(controller_interface arctos_controller.xml)

install(TARGETS arctos_segment_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES arctos_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(arctos_segment_controller)
ament_export_dependencies(controller_interface rclcpp rclcpp_action pluginlib control_msgs trajectory_msgs arctos_hardware_interface)

ament_package()
