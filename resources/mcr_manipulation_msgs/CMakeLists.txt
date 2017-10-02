cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    moveit_msgs
    mcr_common_msgs
)

add_message_files(
  FILES
    ComponentWiseCartesianDifference.msg
    GraspList.msg
    JointDistance.msg
    SphericalSamplerParameters.msg
    GripperCommand.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    moveit_msgs
    mcr_common_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    moveit_msgs
    mcr_common_msgs
)
