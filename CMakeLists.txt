cmake_minimum_required(VERSION 3.16)
project(realsense_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(realsense_utils_component SHARED
  src/infra_gain_correction_node.cpp
  src/realsense_splitter_node.cpp
)
target_compile_definitions(realsense_utils_component
  PRIVATE COMPOSITION_BUILDING_DLL
)
target_include_directories(realsense_utils_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(realsense_utils_component
  cv_bridge
  message_filters
  rclcpp
  rclcpp_components
  realsense2_camera_msgs
  sensor_msgs
)
target_link_libraries(realsense_utils_component
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(
  realsense_utils_component
  "realsense_utils::InfraGainCorrectionNode"
)
rclcpp_components_register_nodes(
  realsense_utils_component
  "realsense_utils::RealsenseSplitterNode"
)

add_executable(realsense_splitter_node
  src/realsense_splitter_node_main.cpp
)
target_link_libraries(realsense_splitter_node
  realsense_utils_component
)

install(
  TARGETS
    realsense_utils_component
    realsense_splitter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(realsense_utils_component)
ament_export_dependencies(
  cv_bridge
  message_filters
  rclcpp
  rclcpp_components
  realsense2_camera_msgs
  sensor_msgs
)

ament_package()
