# socrob alias: used to save time in typing commands
source ${HOME}/scripts/utils.sh

# socrob robocup alias
alias kill_all_ros_nodes='sudo pkill -f ros'
alias gazebo='rosrun gazebo_ros gzclient' # run gazebo gui client (server must be running)
alias teleop='roslaunch mbot_teleop_joypad teleop_joypad.launch' # teleoperation with joypad
alias smach_viewer='rosrun smach_viewer smach_viewer.py'
alias rviz='rosrun rviz rviz'
alias tf_view_frames='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &' # view current frames
alias cb='catkin build'
alias cbt='catkin build --this'

# tools alias
alias clean='find . -name "*~" -type f -exec /bin/rm -fv -- {} +' # to clean temp files *.~ recursively
alias sl='ls' # when typing fast sometimes ls gets typed as sl
alias cap='pygmentize -g' # replace cat with python-pygments to cat with colors
alias ..='cd .. && ls' # going back one directory and showing files convenient alias
alias m='wmctrl -r :ACTIVE: -b toggle,maximized_vert,maximized_horz' # toggle terminal from restored to maximized

# alias to power off pc
alias poweroff='sudo shutdown -h now'
