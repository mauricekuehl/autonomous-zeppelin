cmake_minimum_required(VERSION 3.5)
project(tfmini_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -lpigpio -lrt -lpthread)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tfmini_ros_node src/TFmini_ros_node.cpp src/TFmini.cpp )
ament_target_dependencies(tfmini_ros_node rclcpp sensor_msgs)

target_link_libraries(tfmini_ros_node pigpio)
target_include_directories(
  tfmini_ros_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)


install(TARGETS tfmini_ros_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()