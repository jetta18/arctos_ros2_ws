cmake_minimum_required(VERSION 3.5)
project(arctos_motor_driver)

# Set compiler options with C++17 support
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mks_motor_driver REQUIRED)

# Create library for MKS data types (ROS2-specific conversions)
add_library(mks_data_types SHARED
  src/mks_data_types.cpp
)

target_include_directories(mks_data_types
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(mks_data_types PUBLIC cxx_std_17)

ament_target_dependencies(mks_data_types
  rclcpp
  can_msgs
  mks_motor_driver
)

# Create the ROS2 wrapper for MKS driver
add_library(arctos_mks_driver SHARED
  src/mks_motor_driver.cpp
)

target_link_libraries(arctos_mks_driver
  mks_data_types
  Threads::Threads
)

target_include_directories(arctos_mks_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(arctos_mks_driver PUBLIC cxx_std_17)

ament_target_dependencies(arctos_mks_driver
  rclcpp
  can_msgs
  sensor_msgs
  mks_motor_driver
)

# Create the motor manager library (motor configuration and state management)
add_library(mks_motor_manager SHARED
  src/mks_motor_manager.cpp
)

target_link_libraries(mks_motor_manager
  arctos_mks_driver
  mks_data_types
  Threads::Threads
)

target_include_directories(mks_motor_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(mks_motor_manager PUBLIC cxx_std_17)

ament_target_dependencies(mks_motor_manager
  rclcpp
  can_msgs
  mks_motor_driver
)

# Install targets
install(TARGETS 
    mks_data_types
    arctos_mks_driver
    mks_motor_manager
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies and targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  can_msgs
  sensor_msgs
  mks_motor_driver
  Threads
)

ament_package()