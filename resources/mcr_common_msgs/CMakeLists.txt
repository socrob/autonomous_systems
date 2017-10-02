cmake_minimum_required(VERSION 2.8.3)
project(mcr_common_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
    ClosedInterval.msg
    Float32Stamped.msg
    PoseStampedList.msg
)

generate_messages(
  DEPENDENCIES
  	std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)

