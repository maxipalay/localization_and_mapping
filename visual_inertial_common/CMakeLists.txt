cmake_minimum_required(VERSION 3.16)
project(visual_inertial_common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(OpenCV REQUIRED)  # your OpenCV was already built with CUDA
find_package(GTSAM REQUIRED)


add_library(visual_inertial_common INTERFACE)

target_include_directories(visual_inertial_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
   ${OpenCV_INCLUDE_DIRS}
  ${_gtsam_eigen}
)

ament_target_dependencies(${PROJECT_NAME}
)

set(_gtsam_eigen "${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen")

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${OpenCV_LIBS}
  gtsam
  gtsam_unstable
)

# -------- Install targets + export set (names MUST match) --------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}                  # <- export set name
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

# Install the CMake export file for find_package()
install(
  EXPORT export_${PROJECT_NAME}                   # <- same export set name
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Ament exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)   # <- same export set
ament_export_include_directories(include ${_gtsam_eigen})
ament_export_dependencies(OpenCV sensor_msgs)

ament_package()