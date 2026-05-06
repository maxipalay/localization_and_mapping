cmake_minimum_required(VERSION 3.16)
project(offline_dense_map_fusion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(nvblox REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/session_loader.cpp
  src/fuser.cpp
  src/exporter.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_core
  nvblox::nvblox_lib
  ${OpenCV_LIBS}
  yaml-cpp
)

add_executable(offline_dense_map_fusion_cli
  src/main.cpp
)

target_include_directories(offline_dense_map_fusion_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(offline_dense_map_fusion_cli
  ${PROJECT_NAME}_core
)

install(
  TARGETS ${PROJECT_NAME}_core offline_dense_map_fusion_cli
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
