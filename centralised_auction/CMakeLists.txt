cmake_minimum_required(VERSION 3.5)
project(centralised_auction)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
# Find dependencies
set(PUBLIC_DEPENDENCIES
  geometry_msgs
  rclcpp
  task_msgs
)
set(PRIVATE_DEPENDENCIES
  rcpputils
)
foreach(dep ${PUBLIC_DEPENDENCIES} ${PRIVATE_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

# Build
add_library(${PROJECT_NAME} SHARED
  src/sequential_auction.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${geometry_msgs_TARGETS}
    rclcpp::rclcpp
    ${task_msgs_TARGETS}
  PRIVATE
    rcpputils::rcpputils
)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PUBLIC_DEPENDENCIES})

ament_package()
