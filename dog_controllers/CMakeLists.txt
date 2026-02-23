cmake_minimum_required(VERSION 3.8)
project(dog_controllers)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options("-Wno-unused-variable")
add_compile_options("-Wno-unused-parameter")

# 基础与编译工具
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# ROS 2 核心通讯与控制
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(robot_state_publisher REQUIRED)

# 消息与坐标变换
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OCS2 核心库 (按依赖链排列)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

# OCS2 机器人工具与动力学
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

# 第三方数学与物理库
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(qpoases_vendor REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread log log_setup)

# 自定义项目包
find_package(dog_bringup REQUIRED)

# 控制插件
file(GLOB_RECURSE NMPC_SOURCES CONFIGURE_DEPENDS "src/ocs2_legged_robot/**/*"
     "src/ocs2_legged_robot/*")

add_library(
  dog_nmpc_wbc_controller SHARED
  src/dog_nmpc_wbc_controller.cpp
  src/common/dog_data_bridge.cpp
  src/common/debug_manager.cpp
  src/state_estimation/StateEstimatorBase.cpp
  src/state_estimation/TopicStateEstimator.cpp
  src/state_estimation/KalmanFilterEstimator.cpp
  src/wbc/WbcBase.cpp
  src/wbc/WeightedWbc.cpp
  src/nmpc/NmpcController.cpp
  ${NMPC_SOURCES})

target_include_directories(
  dog_nmpc_wbc_controller
  PRIVATE include/dog_controllers ${EIGEN3_INCLUDE_DIRS}
          ${pinocchio_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ament_target_dependencies(
  dog_nmpc_wbc_controller
  # ROS 2 基础
  rclcpp
  controller_interface
  hardware_interface
  ament_index_cpp
  robot_state_publisher
  # 消息与坐标
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ocs2_msgs
  # OCS2 核心
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_ros_interfaces
  # OCS2 机器人/动力学
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  # 第三方库
  Boost
  Eigen3
  pinocchio
  urdf
  urdfdom
  qpoases_vendor
  angles
  # 自定义包
  dog_bringup)

pluginlib_export_plugin_description_file(controller_interface
                                         dog_controllers.xml)

# topic发布器
add_library(target_trajectories_component SHARED
            src/TargetTrajectoriesPublisher.cpp)
target_include_directories(target_trajectories_component
                           PUBLIC include/dog_controllers)
ament_target_dependencies(
  target_trajectories_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  ocs2_core
  ocs2_msgs
  ocs2_robotic_tools
  ocs2_ros_interfaces
  Eigen3
  ament_index_cpp)

rclcpp_components_register_nodes(target_trajectories_component
                                 "dog_controllers::TargetTrajectoriesPublisher")
# 安装规则
install(FILES dog_controllers.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS target_trajectories_component dog_nmpc_wbc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
