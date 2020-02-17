cmake_minimum_required(VERSION 2.8.3)
project(rarm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  rarm_msgs
  rarm_framework
)

find_package(rarm_framework REQUIRED)
find_package(Curses REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs rarm_msgs rarm_framework
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

add_executable(rarm_keyboard_node src/rarm_keyboard_control.cpp)
target_link_libraries(rarm_keyboard_node
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)
add_dependencies(rarm_keyboard_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
