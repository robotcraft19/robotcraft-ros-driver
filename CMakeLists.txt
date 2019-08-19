cmake_minimum_required(VERSION 2.8.3)
project(robotcraft2019_driver_g03)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package(
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amazebot src/amazebot.cpp src/amazebot_controller.cpp)
add_executable(square_test src/square_test.cpp src/square_controller.cpp)
add_executable(robot_driver src/robot_driver.cpp)
target_link_libraries(amazebot ${catkin_LIBRARIES})
target_link_libraries(square_test ${catkin_LIBRARIES})
target_link_libraries(robot_driver ${catkin_LIBRARIES})
