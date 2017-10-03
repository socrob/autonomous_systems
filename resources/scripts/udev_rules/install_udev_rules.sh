#!/bin/bash

# add your_username to dialout group, you can confirm this command by executing: "groups your_username"
echo "please execute in your pc : sudo usermod -a -G dialout your_username"

# copy udev rules to udev directory
echo "executing : sudo cp 85-pioneer.rules /etc/udev/rules.d"
sudo cp 85-pioneer.rules /etc/udev/rules.d

# restart udev for the rules to take effect
echo "executing : sudo service udev restart"
sudo service udev restart
