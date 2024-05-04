cmake_minimum_required(VERSION 3.5)
project(avantrex_bot_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

add_library(avantrex_bot_hardware_interface SHARED src/avantrex_bot_hardware_interface.cpp)

target_include_directories(
  avantrex_bot_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(
  avantrex_bot_hardware_interface
  hardware_interface
  controller_manager
  std_msgs
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface avantrex_bot_hardware_interface.xml)

install(
  TARGETS avantrex_bot_hardware_interface
  DESTINATION lib
)

install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  avantrex_bot_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()


