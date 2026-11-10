cmake_minimum_required(VERSION 3.16)
project(igo_nav2_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# During development the unchanged ROS-independent core remains in the ROS1
# repository root. prepare_release.py produces a self-contained package with
# exactly the same files under core/ for an eventual dedicated ROS2 branch.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/core/src/core.cpp")
  set(IGO_CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/core")
else()
  set(IGO_CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
endif()
if(NOT EXISTS "${IGO_CORE_ROOT}/include/igo_local_planner/core.h")
  message(FATAL_ERROR "IGO core missing: use the full repository or prepare_release.py bundle")
endif()
add_library(igo_core STATIC "${IGO_CORE_ROOT}/src/core.cpp")
set_target_properties(igo_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(igo_core PUBLIC
  $<BUILD_INTERFACE:${IGO_CORE_ROOT}/include>
  $<INSTALL_INTERFACE:include>)

add_library(igo_nav2_controller SHARED src/controller.cpp)
target_include_directories(igo_nav2_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(igo_nav2_controller igo_core)
ament_target_dependencies(igo_nav2_controller
  geometry_msgs nav_msgs nav2_core nav2_costmap_2d pluginlib
  rclcpp rclcpp_lifecycle rcl_interfaces std_msgs tf2 tf2_ros tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core igo_nav2_plugin.xml)

install(TARGETS igo_core igo_nav2_controller EXPORT export_igo_nav2_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES "${IGO_CORE_ROOT}/include/igo_local_planner/core.h"
  DESTINATION include/igo_local_planner)
install(FILES README.md LICENSE CHANGELOG.rst DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})

add_executable(igo_nav2_plugin_load_check test/plugin_load_check.cpp)
ament_target_dependencies(igo_nav2_plugin_load_check nav2_core pluginlib)
install(TARGETS igo_nav2_plugin_load_check RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_prepare_release
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_prepare_release.py")
  add_executable(igo_core_scenarios "${IGO_CORE_ROOT}/test/scenarios.cpp")
  target_link_libraries(igo_core_scenarios PRIVATE igo_core)
  add_test(NAME igo_core_scenarios COMMAND igo_core_scenarios)
  add_test(NAME igo_nav2_plugin_load_check COMMAND igo_nav2_plugin_load_check)
  set_tests_properties(igo_core_scenarios PROPERTIES TIMEOUT 300)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_controller.cpp")
    ament_add_gtest(test_igo_nav2_controller test/test_controller.cpp TIMEOUT 300)
    target_link_libraries(test_igo_nav2_controller igo_nav2_controller)
    ament_target_dependencies(test_igo_nav2_controller
      geometry_msgs nav_msgs nav2_core nav2_costmap_2d pluginlib
      rclcpp rclcpp_lifecycle rcl_interfaces std_msgs tf2 tf2_ros tf2_geometry_msgs)
  endif()
endif()
ament_export_targets(export_igo_nav2_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs nav_msgs nav2_core nav2_costmap_2d pluginlib
  rclcpp rclcpp_lifecycle rcl_interfaces std_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_package()
