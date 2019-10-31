# socrob alias: used to save time in typing commands
source ${HOME}/scripts/utils.sh

export MBOT_IP=10.2.0.23   # mbot5
export MBOT_IP_7=10.2.0.21  # mbot7

# socrob robocup alias
alias kill_all_ros_nodes='sudo pkill -f ros'
alias launch_mbot_ros_driver_manually='source /opt/mbot_ros/setup.bash && roslaunch /opt/mbot_ros/share/mbot_ros/mbot.launch'
alias bringup_mbot='roslaunch mbot_bringup robot.launch'
alias bringup_sim='roslaunch mbot_simulation robot.launch'
alias bat='rostopic echo /batteries_voltage' # check battery level on the robot
alias gazebo='rosrun gazebo_ros gzclient' # run gazebo gui client (server must be running)
alias teleop='roslaunch mbot_teleop_joypad teleop_joypad.launch' # teleoperation with joypad
alias moveit='roslaunch mbot_moveit_ist_left_arm move_group.launch' # launch moveit interface
alias rqt_mbot_actions='roslaunch mbot_actions rqt_action_client.launch' # gui to interact with action servers
alias rqt_rosplan='rqt --standalone rosplan_rqt.dispatcher.ROSPlanDispatcher' # knowledge base visualization
alias rqt_planning_coordinator='roslaunch mir_planning_core planning_coordinator_event_gui.launch' # task planning sm manual interface
alias kill_arm='rosrun cyton_gamma_1500_driver set_arm_torque_off' # kill all arm joints alias
alias rqt_moveit='rqt --standalone mcr_moveit_commander_gui --force-discover' # moveit commander custom gui interface
alias smach_viewer='rosrun smach_viewer smach_viewer.py'
alias rviz='rosrun rviz rviz --display-config ${ROS_WORKSPACE}/isr_monarch_robot/mbot_tools/rviz_configurations/universal.rviz'
alias reconfigure='rosrun rqt_reconfigure rqt_reconfigure' # open dynamic reconfigure
alias tf_view_frames='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &' # view current frames
alias cb='catkin build'
alias cbt='catkin build --this'
alias rotate_touchscreen='bash ${ROS_WORKSPACE}/isr_monarch_robot/mbot_drivers/idmind_drivers/idmind_config/ros/scripts/rotate_screen.sh'
alias gen_urdf='rosrun xacro xacro.py `rospack find mbot_description`/urdf/mbot_body.urdf.xacro -o `pwd`/generated.urdf'
alias open_network_manager='sudo nm-connection-editor' # needs the ssh connection to be made via ssh -X, opens gui for network manager
alias enable_object_training='source ~/scripts/enable_object_trainning.sh'
alias create_ros_pkg='source ~/autonomous_systems/resources/scripts/create_ros_pkg.sh'

# tools alias
alias clean='find . -name "*~" -type f -exec /bin/rm -fv -- {} +' # to clean temp files *.~ recursively
alias sl='ls' # when typing fast sometimes ls gets typed as sl
alias cap='pygmentize -g' # replace cat with python-pygments to cat with colors
alias ..='cd .. && ls' # going back one directory and showing files convenient alias
alias m='wmctrl -r :ACTIVE: -b toggle,maximized_vert,maximized_horz' # toggle terminal from restored to maximized
alias pull='git pull origin kinetic' # update socrob repository, works only for those which default branch is kinetic and if your remote name is origin

# network testing alias
alias hello_pub='rostopic pub /test std_msgs/String hello' # publish a test topic to test network sanity
alias hello_sub='rostopic echo /test' # echo a test topic to test network sanity

# component alias
alias dwa='roslaunch mbot_2dnav 2dnav.launch' # launch navigation stack in dwa mode
alias skynet='rosrun mbot_demos planning_coordinator_sm_node' # run planning state machine

# text editor alias
alias e='emacs -nw' # open emacs text editor in terminal mode
alias se='sudo emacs -nw' # open emacs text editor with admin rights
alias sublime='sublime-text'
alias remove_endline_spaces="sed -i 's/\s*$//'" # remove automatically spaces at the end of files, needs the file as argument at the end, i.e. remove_spaces my_file.txt

# alias to power off pc
alias poweroff='sudo shutdown -h now'

# ssh alias
alias mbot='ssh socrob@$MBOT_IP' # alias to quickly ssh into nav robot pc
alias mbot7='ssh socrob@$MBOT_IP_7' # alias to quickly ssh into hri robot pc
alias harode='ssh harode@10.0.2.69' # alias to quickly ssh into harode workstation
alias harodeipv6='ssh -6 -X harode@harode01.ipv6.isr.ist.utl.pt' # ssh harode pc from outside ist

# export ROS_MASTER_URI alias
alias export_mbot='export ROS_MASTER_URI=http://$MBOT_IP:11311 && export ROS_IP=`get_interface_that_pings $MBOT_IP | get_ip_of_interface`'
alias export_mbot7='export ROS_MASTER_URI=http://$MBOT_IP_7:11311 && export ROS_IP=`get_interface_that_pings $MBOT_IP_7 | get_ip_of_interface`'
alias export_harode='export ROS_MASTER_URI=http://10.0.2.69:11311'
alias export_dante='export ROS_MASTER_URI=http://10.0.1.23:11311'

# ERL alias
alias refbox_server='roslaunch roah_rsbb roah_rsbb.launch'
alias refbox_client='roslaunch roah_rsbb_comm_ros test.launch team_name:=SocRob robot_name:=mbot05 rsbb_key:=EKY3GZUe rsbb_host:=10.0.255.255'
