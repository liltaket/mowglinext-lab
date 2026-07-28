cmake_minimum_required(VERSION 3.8)
project(mowgli_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)

add_library(safety_detector src/safety_detector.cpp)
target_include_directories(safety_detector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_library(safety_supervisor_node src/safety_supervisor_node.cpp)
target_include_directories(safety_supervisor_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(safety_supervisor_node safety_detector)
ament_target_dependencies(safety_supervisor_node rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs mowgli_interfaces)

add_executable(safety_supervisor_node_main src/safety_supervisor_main.cpp)
set_target_properties(safety_supervisor_node_main PROPERTIES OUTPUT_NAME safety_supervisor_node)
target_link_libraries(safety_supervisor_node_main safety_supervisor_node)
ament_target_dependencies(safety_supervisor_node_main rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS safety_detector safety_supervisor_node safety_supervisor_node_main ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_detector test/test_safety_detector.cpp)
  target_link_libraries(test_safety_detector safety_detector)
endif()
ament_package()
