cmake_minimum_required(VERSION 3.16)
project(topic_echo_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(topic_echo_gui src/main.cpp)

target_include_directories(topic_echo_gui PRIVATE
  /opt/ros/jazzy/include
)

ament_target_dependencies(topic_echo_gui
  rclcpp
  sensor_msgs
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
)

target_link_libraries(topic_echo_gui Qt5::Widgets)

install(TARGETS topic_echo_gui
  DESTINATION lib/topic_echo_gui
)

install(FILES package.xml
  DESTINATION share/topic_echo_gui
)

ament_package()
