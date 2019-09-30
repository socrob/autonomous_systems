Autonomous Systems
===

This repository holds code and resources for the Autonomous Systems course that takes place at Instituto Superior Tecnico, Lisboa.

For this course you will need to have a machine running Ubuntu, ROS and some other things, please follow the instructions bellow before the laboratory classes begin.

Ubuntu instalation
===
For the first classes you may use a virtual machine (in that case use this [image](http://soma.isr.ist.utl.pt/irsgroup/saut/ubuntu_16_04_64bit.vdi)), but it is advisable, at some point, to use a native instalation of  [Ubuntu 16.04 64-bit](https://www.ubuntu.com/download/alternative-downloads) in order to prevent unwanted issues that may arise from the use of a virtual machine.

ROS Kinetic installation
===
It is advisable to go for the Desktop-Full Instalation of ROS, please go through the instructions in the following link:
http://wiki.ros.org/kinetic/Installation/Ubuntu

Git and Repository setup
===

Install git on your system:

        sudo apt-get install git

Navigate to your home folder:

        cd $HOME

Clone the repository:

        git clone https://github.com/socrob/autonomous_systems.git

Whenever the repo is updated you can get the latest updates with:
        
        cd $HOME/autonomous_systems
        git pull origin master

(Optional) Terminator is a usefull tool when you want to have several terminals open. If you want to install it just run:

        sudo apt-get install terminator

Setup catkin workspace and scripts structure (one time only)
===

A partial documentation of this can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and [here](http://catkin-tools.readthedocs.io/en/latest/index.html)

For convenience and to save you time you can run a script to perform the instructions on this block by doing:

        bash ~/autonomous_systems/resources/auton_sys_setup.sh

If you have used the automated script you can skip to section Lab1 below.

If you decide to do it manually (for didactic purposes) then please continue:

1. Create a folder for your ros workspace in your system:

        mkdir -p ~/ros_ws/src

2. Install python catkin tools (to be able to compile with the new catkin build system, instead of catkin_make which is the old one)

        sudo apt-get install python-catkin-tools

3. Source (enable) ROS on your system (only one time, this line should not be there on your .bashrc as it will be bashed by the scripts structure)

        source /opt/ros/kinetic/setup.bash

4. Compile your workspace:

        cd ~/ros_ws && catkin build

5. Add lines of code to your .bashrc to source the scripts configuration files

        echo "# personal config starts here" >> ~/.bashrc && echo "source ~/scripts/permanent.sh" >> ~/.bashrc

6. Copy scripts folder to your home directory:

        cp -r ~/autonomous_systems/resources/scripts $HOME

7. Source your new scripts structure:

        source ~/.bashrc

Lab1
===

Please open lab1 folder to access the lab slides and follow the instructions for the practical exercise. There is README.md file inside. To save time in class you can go through step 1 in advance.

Lab2
===
Please open lab2 folder to access the lab slides and follow the instructions for the practical exercise. There is README.md file inside.

Questions
===

Please post your questions under:

        https://github.com/socrob/autonomous_systems/issues

By creating an issue, so that we can all benefit from the answers.

Thanks!

Extra Resources
===

[Learn how to program in python, google developers nice online course](https://www.youtube.com/watch?v=tKTZoB2Vjuk&list=PL123FD827C7984559)

[ROS python example on how to publish / subcribe from code](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)

[ROS beginner tutorials](http://wiki.ros.org/ROS/Tutorials)

[Nice ROS online course from ETH Zurich](http://www.rsl.ethz.ch/education-students/lectures/ros.html) see their youtube lectures [here](https://www.youtube.com/watch?list=PLE-BQwvVGf8HOvwXPgtDfWoxd4Cc6ghiP&v=0BxVPCInS3M)

[ROS tf tranformations library package](http://wiki.ros.org/tf)

[ROS tf tutorials](http://wiki.ros.org/tf/Tutorials)

[ROS AMCL package Adaptive Monte Carlo localization](http://wiki.ros.org/amcl)

[ROS gmapping : SLAM algorithm (simultaneous localization and mapping)](http://wiki.ros.org/gmapping)
