# This messages represents the distance between two
# joint position values, for a list of joints.
#
# The message contains a list of joints (revolute or prismatic) defined by:
#  * the name to identify each joint uniquely,
#  * the distance between two joint position values (rad or m)
#
# All arrays in this message have the same size.
# The order of the joint names matches the order of the joint distances.

string[] name
float64[] distance