cmake_minimum_required(VERSION 3.12)
project(turtle_go_figures)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(square src/square_node.cpp)
ament_target_dependencies(square rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(circle src/circle_node.cpp)
ament_target_dependencies(circle rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS
  square
  circle
  DESTINATION lib/${PROJECT_NAME})


ament_package()
