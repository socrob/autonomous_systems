#!/bin/sh

# this shows an example on how to override your settings over the ones set by scripts bash structure

# to configure external mac german layout keyboard (example on how to call other scripts)
alias mackey='source ~/scripts/personal_config/external_mac_german_keyboard_layout.sh'

# enable programs to be searched from any terminal
export PATH=$PATH:~/Software/simlinks

# set by default the robot to have arm (here we override the value from default:mbot05 to custom:mbot05-with-arm)
export ROBOT=mbot05-with-arm

# robot environment set to simulation by default (override ROBOT_ENV default value of isr-testbed to be isr-testbed-sim on this PC)
export ROBOT_ENV=isr-testbed-sim

# rosed with kate instead of vim (override EDITOR to be kate instead of vim)
export EDITOR=kate

# ros workspace, choose only one:

# source ros workspace

# scripts bash structure setd by default the ROS_WORKSPACE env in $HOME/ros_ws/src, however my
# catkin worskpace is on another location, here is an example on how you can override its value
# but at the same time keeping clean your scripts repo
export ROS_WORKSPACE=~/ros_ws/monarch_ws/src
