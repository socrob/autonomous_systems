lab2
===

In today's lab we are going to work with the real hardware: [pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)

The objective of this lab is for you to get familiar with the following tools:

- [pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)
- [tf](http://wiki.ros.org/tf)
- [URDF](http://wiki.ros.org/urdf)
- [robot state publisher](http://wiki.ros.org/robot_state_publisher)
- [AMCL](http://wiki.ros.org/amcl)
- [Mapping](http://wiki.ros.org/gmapping)
- [rosbag](http://wiki.ros.org/rosbag)

Please update your repository to get the new lab2 material:

        cd $HOME/autonomous_systems
        git pull origin master

tf
===

The following tutorial will help you to understand the [tf](http://wiki.ros.org/tf) ros library.

This [video](https://www.youtube.com/watch?v=2gVo06HR2Tc) from the creator of the tf library is very useful, however a little bit advanced.
Make sure you understand the basics of tf before watching it.

Some slides will be presented to you by Oscar Lima.

tf simple example
===

You will need 5 terminals for the next exercise, we advise you to use terminator

        sudo apt-get install terminator

Open terminator and configure to have 5 terminals

        right mouse click -> split vertically
        right mouse click -> split horizontally ...etc

run the following commands on each terminal:

        roscore
        rosrun tf static_transform_publisher 3 2 0 0 0 0 map odom 50
        rosrun tf static_transform_publisher 3 1 0 0 0 0 odom base_link 50
        rosrun tf tf_echo map base_link
        rosrun rviz rviz

configure rviz:

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

Pioneer robot driver installation
===

You can find the pioneer datasheet in the [resources folder](https://github.com/socrob/autonomous_systems/tree/master/resources)

There are Open source C++ development libraries ([ARIA](http://www.mobilerobots.com/Software/ARIA.aspx)) provided by the manufacturer to control the robot via serial port. These libraries are ROS independent, however there is also a ROS wrapper available here : [rosaria ROS wrapper](http://wiki.ros.org/ROSARIA)

To install ARIA on your system (from [mobile robots ARIA wiki](http://robots.mobilerobots.com/wiki/ARIA)) do:

        cd $HOME/Downloads
        wget http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.1a+ubuntu16_amd64.deb
        sudo dpkg -i libaria_2.9.1a+ubuntu16_amd64.deb

To install the ros wrapper:

        source ~/.bashrc
        cd $ROS_WORKSPACE
        git clone https://github.com/amor-ros-pkg/rosaria.git
        catkin build rosaria
        source ~/.bashrc

Install udev rules on your system:

Udev rules have 2 purposes: a) they create a simlink to the physical device b) they provide with adecuate admin rights to write to the serial port 

        cd $HOME/autonomous_systems/resources/scripts/udev_rules/install_udev_rules.sh
        ./install_udev_rules.sh

Ensure that your rules have been properly installed:

Disconnect and connect the robot USB to serial converter cable

        ls /dev/pioneer/usb_to_serial_port

The file should exist! (is a simlink to the port with write permissions)

NOTE: If the above udev rules did not work, you could always do:

        sudo chmod a+rw /dev/ttyUSB0

Keep in mind this last one has the disadvantage that you need to do it every time you unplug-plug the usb to serial adaptor.

Your pioneer port
===

Should be one of this two:

        /dev/ttyUSB0
        /dev/pioneer/usb_to_serial_port

Depending on which branch of the tutorial you have decided (or managed) to follow.

Test your pioneer installation
===

Keep in mind that will need several terminals to complete this steps:

        roscore
        rosparam set RosAria/port /dev/pioneer/usb_to_serial_port && rosrun rosaria RosAria

Move the robot:

        rostopic pub -r 10 /RosAria/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

Open rviz and visualize sonar data:

        rosrun rviz rviz

Pioneer robot teleoperation
===

For this exercise you will need two laptops connected to the same network, one connected to the robot via usb cable (robot laptop) and another one

from which the teleoperation node will be running (command laptop).

Get the ip adress of the robot laptop:

        ifconfig

Look the value in front of "inet addr", you are going to use it in the next step.
        
Make sure you can ping the robot laptop from the command laptop. (replace IP_ADDRESS_OF_ROBOT_LAPTOP with the value obtained from previous step):

        ping IP_ADDRESS_OF_ROBOT_LAPTOP

Tell the command laptop that the roscore is running on another PC (replace IP_ADDRESS_OF_ROBOT_LAPTOP with the value obtained from previous step):

        export ROS_MASTER_URI=http://IP_ADDRESS_OF_ROBOT_LAPTOP:11311

Run the robot driver on the robot laptop (needs two terminals):

        roscore
        rosparam set RosAria/port /dev/pioneer/usb_to_serial_port && rosrun rosaria RosAria
        
Test moving the robot from the command laptop:

        rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
        
If the robot moves then stop it: do ctrl + c on the "rostopic pub terminal" (to kill the process) and run the teleoperation with keyboard:

Make sure the teleoperation node is installed:

        sudo apt-get install ros-kinetic-teleop-twist-keyboard

Run the teleoperation:

        rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel

Move the robot by pressing the following keys on your keyboard:

        u    i    o
        j    k    l
        m    ,    .

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
