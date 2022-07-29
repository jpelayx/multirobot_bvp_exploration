cmake_minimum_required(VERSION 3.0.2)
project(coordinator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  nav_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES coordinator
 CATKIN_DEPENDS roscpp tf2_ros nav_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/Map.cpp src/Coord.cpp src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
