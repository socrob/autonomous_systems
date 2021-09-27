Lab2
===

In today's lab we are going to work with the real hardware: [pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)


The objective of this lab is for you to get familiar with the following tools:

- [pioneer robot](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx)
- [tf](http://wiki.ros.org/tf)
- [URDF](http://wiki.ros.org/urdf)
- [robot state publisher](http://wiki.ros.org/robot_state_publisher)

**Note:** If you will work with [RAPOSA-NG](http://socrob-archive.isr.ist.utl.pt/dokuwiki/doku.php?id=socrobrescue:socrobrescue) or [MBot](https://irsgroup.isr.tecnico.ulisboa.pt/projects/socrob-home/), you can find information in the [wiki](https://github.com/socrob/autonomous_systems/wiki) 


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

        cd $HOME/autonomous_systems/lab2-real-robot
        python tf_listener_tutorial.py

Check the code and make sure you understand each line of the provided code.

Print the tf tree pdf:

        sudo apt-get install ros-kinetic-tf2-tools
        tf_view_frames

Keep in mind this last command (tf_view_frames) is an alias for:

        cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &

The above command goes to a temp folder, runs tf2_tools view_frames.py node that generates a pdf, then simply open the frames.pdf generated with a pdf viewer (evince in this case).

Aditionally at home you can make the [tf tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29) if you want.

Turtlebot Robot
===

Please follow the instructions for what you need to do in the introduction to robotics page [here](https://irob-ist.github.io/introduction-robotics/docs/lab2/).
