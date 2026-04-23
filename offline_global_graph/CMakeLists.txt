cmake_minimum_required(VERSION 3.16)
project(offline_global_graph LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core
  src/session_loader.cpp
  src/optimizer.cpp
  src/exporter.cpp
)

set(_offline_global_graph_gtsam_eigen "${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen")

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${_offline_global_graph_gtsam_eigen}
)

target_link_libraries(${PROJECT_NAME}_core
  gtsam
  yaml-cpp
)

add_executable(offline_global_graph_cli
  src/main.cpp
)

target_include_directories(offline_global_graph_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${_offline_global_graph_gtsam_eigen}
)

target_link_libraries(offline_global_graph_cli
  ${PROJECT_NAME}_core
)

add_executable(offline_global_graph_debug_cli
  src/debug_main.cpp
)

target_include_directories(offline_global_graph_debug_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${_offline_global_graph_gtsam_eigen}
)

target_link_libraries(offline_global_graph_debug_cli
  ${PROJECT_NAME}_core
)

install(
  TARGETS ${PROJECT_NAME}_core offline_global_graph_cli offline_global_graph_debug_cli
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
ament_export_include_directories(include ${_offline_global_graph_gtsam_eigen})

ament_package()
