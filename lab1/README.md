Lab 1
==================

The goal for this lab is to get you introduced with ROS.

For this lab we are going to show you some slides and show some examples of how you can use ROS. If you wish to try some of the commands yourself you can download the simulator we will use: [stage](http://rtv.github.io/Stage/) simulator.

The following instructions have been tested under Ubuntu 16.04 and ROS kinetic.

1. How to install ros stage

    ```bash
    sudo apt install ros-kinetic-stage-ros
    ```

2. Ensure that you are able to go into the package:

    ```bash
    roscd stage
    roscd stage_ros
    ```
   
Run hello_pub package:
===================

In Lab 1 the package hello_pub was developed to demonstrate some ROS Functionalities. To run the package in your computer, first move this folder to your workspace (if you followed the github instructions then it should be called catkin_ws, otherwise replace catkin_ws by the name of your workspace. **You move this folder to ~/catkin_ws/src/**) and then you compile the package by running **catkin build** in a terminal. Then, source your workspace by running:

    source ~/catkin_ws/devel/setup.bash

Finally, open 4 terminals and run, in each one:

Terminal 1 (launch roscore):

    roscore
    
Terminal 2 (launch stage simulator):

    rosrun stage_ros stageros $(rospack find stage)/worlds/pioneer_walle.world

    
Terminal 3 (launch hello_pub node):

    rosrun hello_pub my_node.py

    
Terminal 4: Send a message *"hello"* to the topic **/hello** with the format std_msgs/String. Use the command **rostopic pub *topic* *message_type* *message*** replaced by the right topic/message_type/message.
    
