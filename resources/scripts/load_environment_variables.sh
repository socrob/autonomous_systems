# NETWORK CONFIGURATION

# ROBOT SPECIFIC EXPORTS
if [ $(hostname) == "mbot05n" ]; then
   export ROS_IP=10.1.15.14
elif [ $(hostname) == "mbot05h" ]; then
   export ROS_IP=10.1.15.15
   export ROS_MASTER_URI=http://10.1.15.14:11311
fi

# VOCON NUANCE
export LD_LIBRARY_PATH=$HOME/Software/nuance/bin/linux/i86:${LD_LIBRARY_PATH}
export NUANCE_LIBS=$HOME/Software/nuance/bin/linux/i86

# ROS CONFIG RELATED
export EDITOR=${EDITOR:=vim} # rosed command will open editor using this program
export CATTER='pygmentize -g' # roscat function will display the contents of the file by using this program
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}' # print ros messages on terminal by node name and message
export ROSLAUNCH_SSH_UNKNOWN=1 # to allow running nodes without being a known host

# ROBOT DESCRIPTION AND ROBOT ENVIRONMENT
export ROBOT=${ROBOT:=mbot05} # set only if it does not previously exists, possible values: mbot05-with-arm, mbot05
export ROBOT_ENV=${ROBOT_ENV:=isr-lab} # set only if it does not previously exists

# workaroung to remove gazebo error msgs:
# Error [Param.cc:181] Unable to set value [1,0471975511965976] for key[horizontal_fov]
# Error [Param.cc:181] Unable to set value [0,100000001] for key[near]
export LC_NUMERIC=C
