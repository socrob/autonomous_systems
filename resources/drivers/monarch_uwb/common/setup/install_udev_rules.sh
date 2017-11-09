#!/bin/bash

# modify here for the name of your udev rules filename
UDEV_FILENAME=85-uwb

# get username from environment variable USER
USERNAME=`echo $USER`

# # add socrob use to dialout group, you can confirm this command by executing: "groups socrob"
echo "executing : sudo usermod -a -G dialout ${USERNAME}"
sudo usermod -a -G dialout ${USERNAME}

# copy udev rules to udev directory
echo "executing : sudo cp ${UDEV_FILENAME}.rules /etc/udev/rules.d"
sudo cp ${UDEV_FILENAME}.rules /etc/udev/rules.d

# restart udev for the rules to take effect
echo "executing : sudo service udev restart"
sudo service udev restart

# inform user that he needs to unplug and plug USB device for udev to take action
# for the first time
echo "Your udev rules were installed, however you need to plug out and in again your USB device !"
