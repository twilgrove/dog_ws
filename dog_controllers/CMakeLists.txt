cmake_minimum_required(VERSION 3.8)
project(dog_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options("-Wno-unused-variable")
add_compile_options("-Wno-unused-parameter")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread log log_setup)
find_package(ocs2_core REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(Eigen3 REQUIRED)

# 测试控制插件
add_library(
   dog_nmpc_wbc_controller SHARED
  src/dog_nmpc_wbc_controller.cpp src/dog_data_bridge.cpp
  src/state_estimation/StateEstimatorBase.cpp)
target_include_directories(dog_nmpc_wbc_controller PRIVATE include/dog_controllers
                                                       ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  dog_nmpc_wbc_controller
  rclcpp
  controller_interface
  hardware_interface
  ocs2_core
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  Boost
  Eigen3)
pluginlib_export_plugin_description_file(controller_interface
                                         dog_controllers.xml)

# 安装规则
install(TARGETS dog_nmpc_wbc_controller DESTINATION lib)
install(FILES dog_controllers.xml DESTINATION share/${PROJECT_NAME})

ament_package()
