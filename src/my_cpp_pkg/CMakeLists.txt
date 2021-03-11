cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

add_executable(move_turtle src/move_turtle.cpp)
ament_target_dependencies(move_turtle rclcpp geometry_msgs)

add_executable(rotate_turtle src/rotate_turtle.cpp)
ament_target_dependencies(rotate_turtle rclcpp geometry_msgs)

add_executable(turtlebot src/go_to_goal.cpp)
ament_target_dependencies(turtlebot rclcpp geometry_msgs turtlesim)

add_executable(spawner src/spawner.cpp)
ament_target_dependencies(spawner rclcpp turtlesim my_robot_interfaces)

install(TARGETS
  cpp_node
  robot_news_station
  smartphone
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client
  hw_status_publisher
  move_turtle
  rotate_turtle
  turtlebot
  spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
