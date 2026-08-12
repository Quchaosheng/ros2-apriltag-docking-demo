cmake_minimum_required(VERSION 3.8)
project(demo2_apriltag_docking_cpp)

find_package(ament_cmake REQUIRED)

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

install(
  TARGETS ${PROJECT_NAME}_core
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_quality_policy test/quality_policy_test.cpp)
  target_link_libraries(test_quality_policy ${PROJECT_NAME}_core)
  target_include_directories(test_quality_policy PRIVATE include)
  ament_add_gtest(test_tag_policy test/tag_policy_test.cpp)
  target_link_libraries(test_tag_policy ${PROJECT_NAME}_core)
  target_include_directories(test_tag_policy PRIVATE include)
endif()

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
