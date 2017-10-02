# This message specifies the parameters, and constraints,
# of a pose that is sampled around an object.
#
# A use case of this message is for generating a list of poses
# of the robot's end-effector around an object's pose, for instance,
# as potential (pre-)grasp poses.
#
# The rotations are performed about an axis of a moving frame 'M'
# (e.g. the end-effector's pose). This frame 'M' has the same
# position as the object but with the orientation of the 'base_link'
# frame. The sampled poses start at a distance above the object, as
# specified by the 'height' parameter, then the first rotation rotates
# frame 'M' about Y by the angle 'zenith', then about X by the angle
# 'azimuth', and finally, about Z by the angle 'yaw'. After the
# rotations, an offset along the 'Z' axis of frame 'M' is defined by
# the 'radial_distance'.
#
# height: The height with respect to the object's frame.
#
# zenith: The zenith angle (polar angle) in spherical coordinates
#        (rotation about the Y axis).
#
# azimuth: The azimuth angle in spherical coordinates
#           (rotation about the X axis).
#
# yaw: The rotation angle about the Z axis of the end effector.
#
# radial_distance: The distance between the object and the end effector.
#
# For each of these parameters there is an interval, that allows
# the specified parameter to be varied up to the limits of the interval.
mcr_common_msgs/ClosedInterval height
mcr_common_msgs/ClosedInterval zenith
mcr_common_msgs/ClosedInterval azimuth
mcr_common_msgs/ClosedInterval yaw
mcr_common_msgs/ClosedInterval radial_distance
