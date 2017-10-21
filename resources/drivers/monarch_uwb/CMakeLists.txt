cmake_minimum_required(VERSION 2.8.3)
project(monarch_uwb)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_python_setup()

### EXECUTABLES
# add_executable(odom_fixer ros/src/odom_fixer.cpp)
# arget_link_libraries(odom_fixer ${catkin_LIBRARIES})
