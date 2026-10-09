cmake_minimum_required(VERSION 3.8)
project(volleyball_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(chassis_controllers REQUIRED)
find_package(volleyball_controller REQUIRED)
find_package(volleyball_tracker REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_core SHARED
  src/interception_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  chassis_controllers
  volleyball_controller
  volleyball_tracker
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

add_executable(volleyball_planner_node
  src/planner_node.cpp
)

target_link_libraries(volleyball_planner_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(volleyball_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  chassis_controllers
  volleyball_controller
  volleyball_tracker
)

install(TARGETS
  ${PROJECT_NAME}_core
  volleyball_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
