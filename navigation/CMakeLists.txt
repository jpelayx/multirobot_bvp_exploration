cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  nav_msgs
  geometry_msgs
  visualization_msgs
)
catkin_package(
  LIBRARIES navigation 
  CATKIN_DEPENDS roscpp
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/Robot.cpp src/PotentialGrid.cpp src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

