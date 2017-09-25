# This script will enable the python pcl object recognition
# training software on your pc, it is not permanently enabled
# because it conflicts with the recognition pipeline itself
# since training is done offline one time only, is no problem
# then to enable python-pcl, train, then disable

# add to PYTHON path the python pcl path
#[ -f "~/.bashrc" ] && source ~/.bashrc
export PYTHONPATH=$PYTHONPATH:${HOME}/Software/python-pcl/build/lib.linux-x86_64-2.7

# change the color of your terminal to make you aware that 
# object training is enabled on that specific terminal
PS1="\e[1;31;40m[OBJECT_TRAINING]\e[0m $PS1"
