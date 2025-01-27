cmake_minimum_required(VERSION 3.15)

project(arena_camera_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ArenaCameraNode.cpp
  src/MultiCamSync.cpp
  src/SingleCamera.cpp
  src/light_arena/deviceinfo_helper.cpp
  src/rclcpp_adapter/pixelformat_translation.cpp
  src/rclcpp_adapter/quilty_of_service_translation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  arena_sdk
  cv_bridge
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::ArenaCameraNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

add_executable(${PROJECT_NAME}_multicam
  src/multicam_sync_node.cpp
)
target_include_directories(${PROJECT_NAME}_multicam PUBLIC
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_multicam
  ${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_multicam
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    camera_intrinsics
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
