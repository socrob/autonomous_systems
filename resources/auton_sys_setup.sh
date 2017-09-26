#!/bin/bash

# this script will configure your system for the autonomous systems course
# it is meant to be a helper, however you are responsible to understand each line that is on this file
#
# IMPORTANT: this script is meant to be executed only one time at the beginning of the semester during lab1
# Running this script more times can have negative effects on your setup.

# ensure that you have not sourced ros kinetic installation on your system
SHOULD_BE_EMPTY=`cat ~/.bashrc | grep kinetic`
if [ "$SHOULD_BE_EMPTY" != "" ]; then
  echo "ROS has been bashed in ~/.bashrc !! -> please remove this line: source /opt/ros/kinetic/setup.bash from your ~/.bashrc ! (or call for help on how to do it)"
  exit
fi

# ensure that this is the first time you run this script
SHOULD_BE_EMPTY=`cat ~/.bashrc | grep 'scripts/permanent.sh'`
if [ "$SHOULD_BE_EMPTY" != "" ]; then
  echo "This script has been called at least one time on this PC already! doing nothing (for your own safety)"
  exit
fi

# tell user that he can proceed
echo "ROS has not been bashed in ~/.bashrc, this is good. Lets continue"

# create a folder for your ros workspace in your system
mkdir -p ~/ros_ws/src

# install python catkin tools (to be able to compile with the new catkin build system, instead of catkin_make which is the old one)
sudo apt-get install python-catkin-tools

# source (enable) ROS on your system (only one time, this line should not be there on your .bashrc as it will be bashed by the scripts structure)
source /opt/ros/kinetic/setup.bash

# compile your workspace
cd ~/ros_ws && catkin build

# add lines of code to your .bashrc to source the scripts configuration files
echo "# personal config starts here" >> ~/.bashrc && echo "source ~/scripts/permanent.sh" >> ~/.bashrc

# copy scripts folder to your home directory
cp -r ~/autonomous_systems/resources/scripts $HOME

# source your new scripts structure
source ~/.bashrc

# tell the user about the tremendous success he has just had
echo 'Success, all good. Now you have a catkin workspace on your system and the scripts structure installed'
