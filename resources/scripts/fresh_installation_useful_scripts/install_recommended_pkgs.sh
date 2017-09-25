#!/bin/bash

# recommended programs for fresh ubuntu installations
#----------------------------------------------------

# create list of packages to install
packagelist=(
    ipython           # python console editor
    vim               # terminal text editor
    exfat-fuse        # to acces exfat hard drives
    xbacklight        # to dim light of the monitor
    git-core          # git
    gitg              # git gui
    gitk              # git gui
    terminator        # terminal with split windows option
    kate              # kde text editor
    tree              # draw (print) folder structure from terminal
    python-pygments   # to cat files in colors
    konsole           # kde terminal with search options
    guake             # fancy drop down terminal
    geany             # text editor
    wmctrl            # to maximize a terminal from cmd line
)

### install debian packages listed in array above
sudo apt-get install -y ${packagelist[@]}
