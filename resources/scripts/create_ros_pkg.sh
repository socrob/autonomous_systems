#!/bin/bash

# check that the number of received arguments are correct
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash create_robocup_pkg.sh name_of_my_awesome_pkg"
    exit
fi

# assign first received argument to variable PKG_NAME
PKG_NAME=${1}
AUTHOR_NAME='Oscar Lima'
AUTHOR_EMAIL='olima@isr.tecnico.ulisboa.pt'
MAINTAINER_NAME='Oscar Lima'
MAINTAINER_EMAIL='olima@isr.tecnico.ulisboa.pt'

# setup file content
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 2.8.3)\nproject(${PKG_NAME})\n\nfind_package(catkin REQUIRED\n  COMPONENTS\n)\n\ncatkin_python_setup()\n\ncatkin_package(\n    CATKIN_DEPENDS\n)"
ROS_INDEPENDAT_CLASS_CONTENT="#!/usr/bin/env python\n\ndef my_generic_sum_function(a, b):\n    \"\"\"\n    TODO!! documentation\n    \"\"\"\n    return a + b"
PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>\n<package>\n  <name>${PKG_NAME}</name>\n  <version>1.0.0</version>\n  <description>\n    TODO!\n  </description>\n\n  <license>GPLv3</license>\n\n  <author email=\"olima_84@yahoo.com\">${AUTHOR_NAME}</author>\n  <maintainer email=\"${MAINTAINER_EMAIL}\">${MAINTAINER_NAME}</maintainer>\n\n  <buildtool_depend>catkin</buildtool_depend>\n\n</package>"
CONFIG_YAML_CONTENT="#TODO!!"
README_CONTENT="${PKG_NAME} documentation"
LAUNCH_FILE_CONTENT="<?xml version=\"1.0\"?>\n<launch>\n\n    <!-- small description about your node -->\n    \n    <node   pkg=\"my_package_name\" type=\"my_node_name\" name=\"my_node_name\"\n            respawn=\"false\" output=\"screen\" args=\"\$(find ${PKG_NAME}_name)/ros/config/my_arg_file.yaml\"/>\n\n</launch>"
SCRIPTS_CONTENT="#!/usr/bin/env python\n\nimport ${PKG_NAME}_ros.${PKG_NAME}\n\nif __name__ == '__main__':\n    ${PKG_NAME}_ros.${PKG_NAME}.main()"
MY_NODE_IMPLEMENTATION_CONTENT=""
MY_TEST_CONTENT=""
SETUP_PY_CONTENT="#!/usr/bin/env python\n\nfrom distutils.core import setup\nfrom catkin_pkg.python_setup import generate_distutils_setup\n\n# for your packages to be recognized by python\nd = generate_distutils_setup(\n  packages=['${PKG_NAME}', '${PKG_NAME}_ros'],\n  package_dir={'${PKG_NAME}': 'common/src/${PKG_NAME}', '${PKG_NAME}_ros': 'ros/src/${PKG_NAME}_ros'}\n)\n\nsetup(**d)"

# create folder structure
mkdir ${PKG_NAME}
cd ${PKG_NAME}
mkdir -p common/src/${PKG_NAME}
mkdir -p ros/config
mkdir -p ros/doc
mkdir -p ros/launch
mkdir -p ros/scripts
mkdir -p ros/src/${PKG_NAME}_ros
mkdir -p ros/test

# create file structure
touch CMakeLists.txt
touch common/src/${PKG_NAME}/__init__.py
touch common/src/${PKG_NAME}/my_ros_independant_class.py
touch package.xml
touch ros/config/config_${PKG_NAME}.yaml
touch ros/doc/README.md
touch ros/launch/${PKG_NAME}.launch
touch ros/scripts/${PKG_NAME}_node
touch ros/src/${PKG_NAME}_ros/__init__.py
touch ros/src/${PKG_NAME}_ros/${PKG_NAME}.py
touch ros/test/${PKG_NAME}_test.py
touch setup.py

# make python node executable:
chmod +x ros/scripts/${PKG_NAME}_node

# fill files with information from parameter section
echo -e $CMAKELISTS_CONTENT  > CMakeLists.txt
echo -e $ROS_INDEPENDAT_CLASS_CONTENT > common/src/${PKG_NAME}/my_ros_independant_class.py
echo -e $PACKAGE_XML_CONTENT > package.xml
echo -e $CONFIG_YAML_CONTENT > ros/config/config_${PKG_NAME}.yaml
echo -e $README_CONTENT > ros/doc/README.md
echo -e $LAUNCH_FILE_CONTENT > ros/launch/${PKG_NAME}.launch 
echo -e $SCRIPTS_CONTENT > ros/scripts/${PKG_NAME}_node
echo -e $MY_NODE_IMPLEMENTATION_CONTENT
echo -e $MY_TEST_CONTENT > ros/test/${PKG_NAME}_test.py
echo -e $SETUP_PY_CONTENT > setup.py
