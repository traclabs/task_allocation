cmake_minimum_required(VERSION 3.5)
project(task_msgs)
set(CMAKE_CXX_STANDARD 14)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} msg/Task.msg msg/TaskArray.msg
                           DEPENDENCIES geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
