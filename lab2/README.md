Lab2
===

In today's lab you are going to work with the real simulator: [TODO: CHOOSE ROBOT pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)


The objective of this lab is for you to get familiar with the following tools:

- [pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)
- [tf](http://wiki.ros.org/tf)
- [URDF](http://wiki.ros.org/urdf)
- [robot state publisher](http://wiki.ros.org/robot_state_publisher)
- [AMCL](http://wiki.ros.org/amcl)
- [Mapping](http://wiki.ros.org/gmapping)
- [rosbag](http://wiki.ros.org/rosbag)


Before the Lab
===
Before the lab you should install the simulator and terminator by typing

    sudo apt-get install smthg
    sudo apt-get install terminator
    


How you can create a new package
===

To create a new package we placed a script in the resources folder, to help you in this process. You just need to modify and run the command bellow:

    ```bash
    source ~/<PATH TO YOUR GIT REPO>/autonomous_systems/resources/scripts/create_ros_pkg.sh <Name of your Awesome Package>
    ```


tf
===

The following tutorial will help you to understand the [tf](http://wiki.ros.org/tf) ros library.

This [video](https://www.youtube.com/watch?v=2gVo06HR2Tc) from the creator of the tf library is very useful, however a little bit advanced.
Make sure you understand the basics of tf before watching it.

tf simple example
===

You will need 5 terminals for the next exercise, we advise you to use terminator. If you didn't install it yet, please run:

        sudo apt-get install terminator

Open terminator and configure it to have 5 terminals

        right mouse click -> split vertically
        right mouse click -> split horizontally ...etc

Run the following commands on each terminal:

        roscore
        rosrun tf static_transform_publisher 3 2 0 0 0 0 map odom 50
        rosrun tf static_transform_publisher 3 1 0 0 0 0 odom base_link 50
        rosrun tf tf_echo map base_link
        rosrun rviz rviz

Configure rviz:

        set fixed frame as map
        add tf topic
        expand tf topic options and set "Marker Scale" to 10

Observe the output of the tf echo terminal:

        Translation: [6.000, 3.000, 0.000]
        there you have the transform!

Notice that the same can be done from code by taking example from here [tf tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29)

A node has being prepared for you as an example of how to use tf library from code:

        cd $HOME/autonomous_systems/lab2
        python tf_listener_tutorial.py

Check the code and make sure you understand each line of the provided code.

Print the tf tree pdf:

        sudo apt-get install ros-kinetic-tf2-tools
        tf_view_frames

Keep in mind this last command (tf_view_frames) is an alias for:

        cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &

The above command goes to a temp folder, runs tf2_tools view_frames.py node that generates a pdf, then simply open the frames.pdf generated with a pdf viewer (evince in this case).

Aditionally at home you can make the [tf tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29) if you want.

Robot Simulator Instalation
===

If you haven´t already done this, you can download the robot simulator from this [link](https://link.com)


Run the Simulation
==

0. Run the simulation by launching roscore in a terminal:

       roscore
       
1. In another terminal launch the gazebo environment and the simulated robot:

       roslaunch TODO
       
2. View the topics launched:

       rostopic list
       
    **Note**: You can search for some specific topic by running
       
       rostopic list | grep <keyword>
       
3. move the robot (rotate):

    ```bash
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
    ```
    
4. move the robot (forward):

    ```bash
    rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ```

5. Analyze a bit the situation:

    * If you want more information about rostopic command, check the following website: http://wiki.ros.org/rostopic

    * Check at which rate is publishing the data:

        ```bash
        rostopic hz /cmd_vel
        ```

    * Echo the data on another terminal:

        ```bash
        rostopic echo /cmd_vel
        ```

6. teleoperate the robot with the keyboard

    ```bash
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel
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

Localize the robot
===

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
