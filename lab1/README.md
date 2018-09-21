Lab 1
==================

The goal for this lab is to get you introduced with ROS.

For this lab we are going to work with [stage](http://rtv.github.io/Stage/) simulator.
The following instructions have been tested under Ubuntu 16.04 and ROS kinetic.

0. To create a new package we placed a script in the resources folder, to help you in this process. You just need to modify and run the command bellow:

        source ~/<PATH TO YOUR GIT REPO>/autonomous_systems/resources/scripts/create_ros_pkg.sh <Name of your Awesome Package>

1. Install ros stage

        sudo apt install ros-kinetic-stage-ros

2. Ensure that you are able to go into the package:

        roscd stage
        roscd stage_ros

3. Explore the stage package, take a look at the bitmaps.
Stage will create worlds based on this 2d bitmaps.

        roscd stage/worlds/bitmaps/
        nautilus .

4. Run stage (in 2 terminals!):

        roscore
        rosrun stage_ros stageros $(rospack find stage)/worlds/pioneer_walle.world
        
If you see the warning: [ WARN] [/stageros]: ROS Stage currently supports rangers with 1 sensor only.
Do not worry, is normal and expected.

You should be able to see now 2 robots: 1 pioneer and 1 wall-e

5. Play around with stage options:

        press R -> perspective camera
        mouse right click -> drag to change the camera position

6. open rviz and visualize some data:

7. move the robot (rotate):

        rostopic pub -r 10 /robot_0/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

8. move the robot (forward):

        rostopic pub -r 10 /robot_0/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

9. Analyze a bit the situation:

If you want more information about rostopic command, check the following website:

        http://wiki.ros.org/rostopic

Check at which rate is publishing the data:

        rostopic hz /robot_0/cmd_vel

Echo the data on another terminal:

        rostopic echo /robot_0/cmd_vel
        
10. teleoperate the robot with the keyboard

        sudo apt-get install ros-kinetic-teleop-twist-keyboard
        rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_0/cmd_vel

move the robot by pressing the following keys on your keyboard:

        u    i    o
        j    k    l
        m    ,    .

11. reset robot positions using service call:

        rosservice call /reset_positions "{}"

12. create a behavior for the pioneer robot so that moves forward until it has an obstacle at 1.0 m then stops and starts rotating to the right until the object is no longer so close. It should loop through this behaviour infinitely.

hint: use the pioneer_behavior.py file (located on this folder) and complete the parts of the code that are missing.

hint: follow the [tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
      it will help you to understand how to publish / subscribe to topics in ros using python code

you can run the file by copying the content of the file pioneer_behavior.py to the node of the package you created in the first step and the run the corresponding executable using rosrun.

13. Watch the following youtube [video](https://youtu.be/IZcE1vrMCvM) to get see an example of
one posible solution for the problem in this lab.

14. Check additional resources:

[About the Pioneer (real) robot](http://www.mobilerobots.com/Libraries/Downloads/Pioneer3DX-P3DX-RevA.sflb.ashx)
