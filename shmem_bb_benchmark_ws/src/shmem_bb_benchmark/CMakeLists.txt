cmake_minimum_required(VERSION 3.20)
project(shmem_bb_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shmem_buffer REQUIRED)
find_package(shmem_buffer_backend REQUIRED)
find_package(shmem_buffer_backend_msgs REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(rosidl_buffer_backend REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${shmem_buffer_backend_INCLUDE_DIRS}
)

# Component libraries
add_library(shmem_test_components SHARED
  src/shmem_image_publisher_node.cpp
  src/shmem_image_subscriber_node.cpp
)

target_include_directories(shmem_test_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(shmem_test_components
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  shmem_buffer::shmem_buffer
  rosidl_runtime_cpp::rosidl_runtime_cpp
)

rclcpp_components_register_nodes(shmem_test_components
  "shmem_bb_benchmark::ShmemImagePublisherNode"
  "shmem_bb_benchmark::ShmemImageSubscriberNode"
)

# Standalone Executables
add_executable(shmem_image_publisher_node src/shmem_image_publisher_main.cpp)
target_link_libraries(shmem_image_publisher_node shmem_test_components rclcpp::rclcpp)

add_executable(shmem_image_subscriber_node src/shmem_image_subscriber_main.cpp)
target_link_libraries(shmem_image_subscriber_node shmem_test_components rclcpp::rclcpp)

# Install rules
install(
  TARGETS
    shmem_test_components
    shmem_image_publisher_node
    shmem_image_subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/benchmark_latency.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
