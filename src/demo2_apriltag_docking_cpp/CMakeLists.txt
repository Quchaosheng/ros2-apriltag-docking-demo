cmake_minimum_required(VERSION 3.8)
project(demo2_apriltag_docking_cpp)

find_package(ament_cmake REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(${PROJECT_NAME}_core
  src/core/quality_policy.cpp
  src/core/tag_policy.cpp
  src/core/task_policy.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_adapters
  src/adapters/dock_spec_loader.cpp
)
target_include_directories(${PROJECT_NAME}_adapters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_adapters PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_tag_pose_adapter
  src/adapters/tag_pose_adapter.cpp
)
target_include_directories(${PROJECT_NAME}_tag_pose_adapter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME}_tag_pose_adapter PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_tag_pose_adapter
  apriltag_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_tag_pose_adapter ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_adapters
  PUBLIC
    ${PROJECT_NAME}_core
    yaml-cpp::yaml-cpp
)

add_executable(tag_pose_bridge_cpp
  src/nodes/tag_pose_bridge.cpp
)
target_compile_options(tag_pose_bridge_cpp PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(tag_pose_bridge_cpp
  apriltag_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(tag_pose_bridge_cpp
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_adapters
  ${PROJECT_NAME}_tag_pose_adapter
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_adapters
    ${PROJECT_NAME}_tag_pose_adapter
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  EXPORT ${PROJECT_NAME}_targets
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  TARGETS tag_pose_bridge_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_quality_policy test/quality_policy_test.cpp)
  target_link_libraries(test_quality_policy ${PROJECT_NAME}_core)
  target_include_directories(test_quality_policy PRIVATE include)
  ament_add_gtest(test_tag_policy test/tag_policy_test.cpp)
  target_link_libraries(test_tag_policy ${PROJECT_NAME}_core)
  target_include_directories(test_tag_policy PRIVATE include)
  ament_add_gtest(test_task_policy test/task_policy_test.cpp)
  target_link_libraries(test_task_policy ${PROJECT_NAME}_core)
  target_include_directories(test_task_policy PRIVATE include)
  ament_add_gtest(test_dock_spec_loader test/dock_spec_loader_test.cpp)
  target_link_libraries(test_dock_spec_loader ${PROJECT_NAME}_adapters)
  target_include_directories(test_dock_spec_loader PRIVATE include)
  ament_add_gtest(test_tag_pose_adapter test/tag_pose_adapter_test.cpp)
  target_link_libraries(test_tag_pose_adapter ${PROJECT_NAME}_tag_pose_adapter)
  target_include_directories(test_tag_pose_adapter PRIVATE include)
endif()

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  apriltag_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  sensor_msgs
  yaml_cpp_vendor
  yaml-cpp
)
ament_package()
