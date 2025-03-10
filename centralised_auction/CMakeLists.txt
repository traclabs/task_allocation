cmake_minimum_required(VERSION 3.5)
project(centralised_auction)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(task_msgs REQUIRED)

# Build
include_directories(include)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED src/sequential_auction.cpp)
ament_target_dependencies(${PROJECT_NAME} geometry_msgs rclcpp rcpputils task_msgs)

# Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(geometry_msgs rclcpp task_msgs)

# Package
ament_package()
