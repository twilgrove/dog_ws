cmake_minimum_required(VERSION 3.8)
project(dog_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(dog_state_msg "msg/DogState.msg" DEPENDENCIES
                           geometry_msgs std_msgs)

install(DIRECTORY launch config xacro meshes DESTINATION share/${PROJECT_NAME})

ament_package()
