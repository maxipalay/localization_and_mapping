cmake_minimum_required(VERSION 3.16)
project(visual_inertial LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visual_inertial_frontend REQUIRED)  # uses the installed export
find_package(visual_inertial_common REQUIRED)
find_package(visual_inertial_localization REQUIRED)
find_package(visual_inertial_optimization REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(eigen3_cmake_module REQUIRED)

add_library(visual_inertial_transport STATIC
  src/transport/common_transport.cpp
  src/transport/frontend_transport.cpp
  src/transport/localization_transport.cpp
  src/transport/optimization_transport.cpp)

add_executable(tracking_node src/tracking_node.cpp)
add_executable(localization_node src/localization_node.cpp)
add_executable(optimization_node
  src/optimization_node.cpp
  src/optimization_node_params.cpp)
add_executable(path_viz_node src/path_viz_node.cpp)
add_executable(tracks_viz_node src/tracks_viz_node.cpp)

target_include_directories(visual_inertial_transport
  PUBLIC include
  PRIVATE ${OpenCV_INCLUDE_DIRS})

target_include_directories(optimization_node PRIVATE include)

ament_target_dependencies(visual_inertial_transport
  builtin_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visual_inertial_common
  visual_inertial_frontend
  visual_inertial_localization
  visual_inertial_optimization)

# Pull include dirs/defs from deps (including your lib)
ament_target_dependencies(tracking_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV visual_inertial_frontend visual_inertial_common tf2_ros)

ament_target_dependencies(tracks_viz_node
  rclcpp sensor_msgs cv_bridge image_transport OpenCV)

ament_target_dependencies(optimization_node
  rclcpp sensor_msgs visual_inertial_common visual_inertial_optimization tf2_ros)

ament_target_dependencies(localization_node
  apriltag_msgs rclcpp sensor_msgs tf2_ros)

ament_target_dependencies(path_viz_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

target_link_libraries(tracking_node
  visual_inertial_frontend::visual_inertial_frontend
  visual_inertial_common::visual_inertial_common
  visual_inertial_transport
)

target_link_libraries(optimization_node
  visual_inertial_optimization::visual_inertial_optimization
  visual_inertial_common::visual_inertial_common
  visual_inertial_transport
)

target_link_libraries(localization_node
  visual_inertial_localization::visual_inertial_localization
  visual_inertial_transport
)

install(TARGETS tracking_node localization_node optimization_node path_viz_node tracks_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrontendHealth.msg"
  "msg/FrontendIntervalHealth.msg"
  "msg/Keyframe.msg"
  "msg/ImuBias.msg"
  "msg/OptimizedKeyframePose.msg"
  "msg/OptimizationStats.msg"
  "msg/OptimizationResult.msg"
  "msg/LocalizationPosePrior.msg"
  "msg/LocalizationCommand.msg"
  "msg/LocalizationFeedback.msg"
  "msg/Tracks.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(visual_inertial_transport "${cpp_typesupport_target}")
target_link_libraries(tracking_node "${cpp_typesupport_target}")
target_link_libraries(localization_node "${cpp_typesupport_target}")
target_link_libraries(tracks_viz_node "${cpp_typesupport_target}")
target_link_libraries(optimization_node "${cpp_typesupport_target}")

ament_package()
