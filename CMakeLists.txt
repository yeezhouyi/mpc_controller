cmake_minimum_required(VERSION 3.16)
project(mpc_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

# ---- Library ----
add_library(mpc_controller SHARED
  src/mpc_controller.cpp
  src/osqp_solver.cpp
  src/linear_model.cpp
)

target_include_directories(mpc_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/3rdparty>
    $<INSTALL_INTERFACE:include/3rdparty>
)

target_link_libraries(mpc_controller
  PUBLIC
    controller_interface::controller_interface
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    std_msgs::std_msgs__rosidl_generator_cpp
    pluginlib::pluginlib
    realtime_tools::realtime_tools
    osqp::osqp
)

pluginlib_export_plugin_description_file(controller_interface mpc_controller_plugins.xml)

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_osqp_solver test/osqp_solver_test.cpp)
  target_link_libraries(test_osqp_solver mpc_controller)
  target_include_directories(test_osqp_solver PRIVATE include include/3rdparty)

  ament_add_gtest(test_mpc_controller test/mpc_controller_test.cpp)
  target_link_libraries(test_mpc_controller mpc_controller)
  target_include_directories(test_mpc_controller PRIVATE include include/3rdparty)
endif()

# ---- Linters ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- Install ----
install(
  TARGETS mpc_controller
  EXPORT mpc_controller
  DESTINATION lib
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install public headers (including 3rdparty wrapper)
install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/trajectory_publisher.py scripts/benchmark_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Export ----
ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  pluginlib
  realtime_tools
  Eigen3
  osqp
)
ament_export_include_directories(include)
ament_export_targets(mpc_controller)

ament_package()
