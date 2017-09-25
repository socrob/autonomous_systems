#!/bin/bash

# add socrob use to dialout group, you can confirm this command by executing: "groups socrob"
echo "executing : sudo usermod -a -G dialout socrob"
sudo usermod -a -G dialout socrob

# copy udev rules to udev directory
echo "executing : sudo cp 85-mbot.rules /etc/udev/rules.d"
sudo cp 85-mbot.rules /etc/udev/rules.d

# restart udev for the rules to take effect
echo "executing : sudo service udev restart"
sudo service udev restart
