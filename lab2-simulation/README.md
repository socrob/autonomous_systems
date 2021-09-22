Lab2
===

In today's lab you are going to work with the real simulator: [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)


The objective of this lab is for you to get familiar with the following tools:

- [turtlebot3 robot](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [robot state publisher](http://wiki.ros.org/robot_state_publisher)
- [AMCL](http://wiki.ros.org/amcl)
- [Mapping](http://wiki.ros.org/gmapping)
- [rosbag](http://wiki.ros.org/rosbag)


Before the Lab
===
Before the lab you should install the simulator and terminator by typing

    sudo apt-get install terminator
    
    sudo apt-get install ros-kinetic-turtlebot3
    
    cd ~/catkin_ws/src
    
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

    catkin build (or cb if you have the aliases)

    source ~/.bashrc (or S if you have the aliases)

Robot Simulator Instalation
===

If you haven´t already done this, you can download the robot simulator as instructed above.


Run the Simulation
==

0. Run the simulation by launching roscore in a terminal:

       roscore
       
1. In another terminal launch the gazebo environment and the simulated robot:

       roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
or

       roslaunch turtlebot3_gazebo turtlebot3_world.launch
       
2. View the topics launched:

       rostopic list
       
    **Note**: You can search for some specific topic by running
       
       rostopic list | grep <keyword>
       
3. View the nodes running:

       rosnode list
       
4. View the parameter server:

       rosparam list
       
5. move the robot (rotate):

       rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
    
6. move the robot (forward):

       rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

7. Analyze a bit the situation:

    * If you want more information about rostopic command, check the following website: http://wiki.ros.org/rostopic

    * Check at which rate is publishing the data:

        ```bash
        rostopic hz /cmd_vel
        ```

    * Echo the data on another terminal:

        ```bash
        rostopic echo /cmd_vel
        ```

8. teleoperate the robot with the keyboard

    ```bash
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
    ```

    * move the robot by pressing the following keys on your keyboard:

        ```
        u    i    o
        j    k    l
        m    ,    .
        ```

rosbag
===

Keep the simulation runing and record a rosbag by moving into the directory you wish to have the rosbag recorded and typing

        rosbag record -a
        
And then move the robot around with teleoperation for some seconds. When you wish to stop just press Ctrl + C in the rosbag record terminal.
        
You can choose the topics you want to record instead of recording everything.

Kill the simulation and all rosnodes, and run your bag by running this command in the rosbag directory

        rosbag play <the name f your rosbag>


rosbag + AMCL
===

For this exercise you will work with pre recorded robot data from the monarch robot.

Please clone the following repository into your catkin workspace (it contains the dataset):

        roscd
        git clone https://github.com/oscar-lima/autom_param_optimization.git
        
Build your catkin workspace:
        
        roscd
        catkin build
        source ~/.bashrc
        
More on turtlebot simulation
===

If you visit the link https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation you will find many tutorials that you can follow to get more used to ROS, gazebo, SLAM, localization, ...

Localize the robot
===
For even more localization practise you can do the following:

Kill all previous ros nodes and start freshly

Install dependencies:

        cp -r $HOME/autonomous_systems/resources/mbot_world_model/ $ROS_WORKSPACE
        cp -r $HOME/autonomous_systems/resources/mcr_states/ $ROS_WORKSPACE
        cp -r $HOME/autonomous_systems/resources/mcr_manipulation_msgs/ $ROS_WORKSPACE
        cp -r $HOME/autonomous_systems/resources/mcr_common_msgs/ $ROS_WORKSPACE
        sudo apt-get install ros-kinetic-map-server ros-kinetic-moveit-msgs ros-kinetic-amcl

Build the dependencies:

        roscd
        catkin build
        source ~/.bashrc

NOTE: Please notice for new pkgs to be recognized by ROS you need to 1) build it 2) source the .bashrc !

Run the server:

        roscore
        roslaunch mbot_autom_tuning_amcl amcl_instance_server.launch

Run the client:

        rosrun mbot_autom_tuning_amcl run_amcl_instance_client_node

Configure rviz to visualize the localization:

See the following [youtube video](https://youtu.be/8Tb2poqgDqM) that explains how to configure rviz
