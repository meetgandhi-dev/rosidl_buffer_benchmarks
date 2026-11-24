cmake_minimum_required(VERSION 3.20)
project(cuda_bb_benchmark)

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
find_package(cuda_buffer REQUIRED)
find_package(cuda_buffer_backend REQUIRED)
find_package(cuda_buffer_backend_msgs REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(rosidl_buffer_backend REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CUDAToolkit_INCLUDE_DIRS}
)

# Component library
add_library(cuda_test_components SHARED
  src/cuda_image_publisher_node.cpp
  src/cuda_image_subscriber_node.cpp
)

target_include_directories(cuda_test_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(cuda_test_components
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  cuda_buffer::cuda_buffer
  ${cuda_buffer_backend_msgs_TARGETS}
  rosidl_runtime_cpp::rosidl_runtime_cpp
  CUDA::cudart
  CUDA::cuda_driver
)

rclcpp_components_register_nodes(cuda_test_components
  "cuda_bb_benchmark::CudaImagePublisherNode"
  "cuda_bb_benchmark::CudaImageSubscriberNode"
)

# Standalone Executables
add_executable(cuda_image_publisher_node src/cuda_image_publisher_main.cpp)
target_link_libraries(cuda_image_publisher_node cuda_test_components rclcpp::rclcpp)

add_executable(cuda_image_subscriber_node src/cuda_image_subscriber_main.cpp)
target_link_libraries(cuda_image_subscriber_node cuda_test_components rclcpp::rclcpp)

# Install rules
install(
  TARGETS
    cuda_test_components
    cuda_image_publisher_node
    cuda_image_subscriber_node
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
    scripts/run_iterations.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
