cmake_minimum_required(VERSION 3.8)
project(demo2_apriltag_docking LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(docking_core
  src/tag_policy.cpp
  src/dock_config.cpp
  src/quality_policy.cpp
  src/task_policy.cpp
)
target_include_directories(docking_core PUBLIC include)
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(docking_core PUBLIC yaml-cpp::yaml-cpp)
else()
  target_link_libraries(docking_core PUBLIC yaml-cpp)
endif()
if(MSVC)
  target_compile_options(docking_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(docking_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(tag_pose_bridge src/tag_pose_bridge.cpp)
target_link_libraries(tag_pose_bridge PRIVATE docking_core)
ament_target_dependencies(tag_pose_bridge
  apriltag_msgs diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

add_executable(docking_task_bridge src/docking_task_bridge.cpp)
target_link_libraries(docking_task_bridge PRIVATE docking_core)
ament_target_dependencies(docking_task_bridge
  action_msgs diagnostic_msgs nav2_msgs rclcpp rclcpp_action std_msgs std_srvs)

install(TARGETS docking_core tag_pose_bridge docking_task_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include config launch models maps worlds
  DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_python_install_package(demo2_apriltag_docking)
install(PROGRAMS demo2_apriltag_docking/bag_quality.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME analyze_docking_bag)

if(BUILD_TESTING)
  ament_add_gtest(cpp_tag_policy_tests test/test_tag_policy.cpp)
  target_link_libraries(cpp_tag_policy_tests docking_core)

  ament_add_gtest(cpp_quality_policy_tests test/test_quality_policy.cpp)
  target_link_libraries(cpp_quality_policy_tests docking_core)

  ament_add_gtest(cpp_task_policy_tests test/test_task_policy.cpp)
  target_link_libraries(cpp_task_policy_tests docking_core)

  file(GLOB PYTHON_TESTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.py")
  foreach(PYTHON_TEST ${PYTHON_TESTS})
    get_filename_component(PYTHON_TEST_NAME ${PYTHON_TEST} NAME_WE)
    ament_add_pytest_test(${PYTHON_TEST_NAME} ${PYTHON_TEST})
  endforeach()
endif()

ament_package()
