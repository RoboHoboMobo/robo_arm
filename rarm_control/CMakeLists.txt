cmake_minimum_required(VERSION 2.8.3)
project(rarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Curses REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/kinematics.cpp src/trans_matrix.cpp)
add_executable(rarm_keyboard_node src/rarm_keyboard_control.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
target_link_libraries(rarm_keyboard_node
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
