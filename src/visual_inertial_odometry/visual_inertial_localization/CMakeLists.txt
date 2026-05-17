cmake_minimum_required(VERSION 3.16)
project(visual_inertial_localization LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/localization.cpp
  src/localization_engine.cpp
  src/localization_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  apriltag_msgs
  geometry_msgs
  rclcpp
  tf2_ros
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  yaml-cpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(
  EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(apriltag_msgs geometry_msgs rclcpp tf2_ros yaml-cpp Eigen3)

if(BUILD_TESTING)
  ament_add_gtest(test_localization_engine
    test/test_localization_engine.cpp)
  target_link_libraries(test_localization_engine
    ${PROJECT_NAME})

  ament_add_gtest(test_localization_controller
    test/test_localization_controller.cpp)
  target_link_libraries(test_localization_controller
    ${PROJECT_NAME})
endif()

ament_package()
