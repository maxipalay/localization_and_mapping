cmake_minimum_required(VERSION 3.16)
project(visual_inertial_bringup LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_mapping_mode_node_graph.py
    ENV
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/test_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs)
  add_launch_test(test/test_localization_mode_node_graph.py
    ENV
      ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/test_ros_home
      ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs)
endif()

ament_package()
