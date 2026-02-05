cmake_minimum_required(VERSION 3.8)
project(dog_controllers)

# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
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
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread log log_setup)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_legged_robot REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)

# 控制插件
add_library(
  dog_nmpc_wbc_controller SHARED
  src/dog_nmpc_wbc_controller.cpp src/common/dog_data_bridge.cpp
  src/state_estimation/StateEstimatorBase.cpp src/common/dog_interface.cpp
  src/state_estimation/TopicStateEstimator.cpp)

target_include_directories(
  dog_nmpc_wbc_controller
  PRIVATE include/dog_controllers ${EIGEN3_INCLUDE_DIRS}
          ${pinocchio_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ament_target_dependencies(
  dog_nmpc_wbc_controller
  rclcpp
  controller_interface
  hardware_interface
  ocs2_core
  ocs2_legged_robot
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_tools
  Boost
  Eigen3
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  urdfdom)

pluginlib_export_plugin_description_file(controller_interface
                                         dog_controllers.xml)

# 安装规则
install(TARGETS dog_nmpc_wbc_controller DESTINATION lib)
install(FILES dog_controllers.xml DESTINATION share/${PROJECT_NAME})

ament_package()
