cmake_minimum_required(VERSION 3.8)
project(arctos_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArctosState.msg"
  "msg/ArctosEndstops.msg"
  "msg/ArctosError.msg"
  "msg/ConnectionStatus.msg"
  "srv/Ping.srv"
  "srv/SetServo.srv"
  "srv/Stop.srv"
  "srv/HomeAxis.srv"
  "srv/HomeToPosition.srv"
  "srv/SetHomeOffset.srv"
  "srv/GetHomeOffset.srv"
  "srv/JogAxis.srv"
  "srv/SetAxisPosition.srv"
  "srv/Reconnect.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
