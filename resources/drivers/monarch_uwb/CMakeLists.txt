cmake_minimum_required(VERSION 2.8.3)
project(monarch_uwb)

find_package(catkin REQUIRED
    std_msgs
    message_generation
)

add_message_files(
  DIRECTORY
    ros/msg
  FILES
    uwb_anchor.msg
    uwb_anchor_array.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)
