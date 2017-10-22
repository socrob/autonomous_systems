#!/usr/bin/env python

import rospy
import std_msgs.msg

from monarch_uwb.uwb import UWBDriver
from monarch_uwb.msg import uwb_anchor, uwb_anchor_array

class UWBDriverNode(object):
    '''
    Ultra Wide Band sensor driver ROS wrapper
    '''
    def __init__(self):
        # to control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 1.0)) # set between 1 - 10
        # get tag id from param server
        self.tag_id = rospy.get_param('~tag_id', 'front')
        # get the tag device port from param server
        device = rospy.get_param('~device', '/dev/mbot/uwb')
        # get the reference frame in which the readings are comming from
        frame_id = rospy.get_param('~frame_id', self.tag_id + '_uwb_' + 'link')
        # create object of uwb class
        self.tag = UWBDriver(device)
        # setup uwb readings publisher
        self.uwb_pub = rospy.Publisher('~' + self.tag_id + '_tag', uwb_anchor_array, queue_size=1)
        # create single anchor msgs
        self.anchorA_msg = uwb_anchor()
        self.anchorB_msg = uwb_anchor()
        self.anchorC_msg = uwb_anchor()
        # fill msg with constant info (that will not change)
        self.anchorA_msg.anchor_id = 'A'
        self.anchorB_msg.anchor_id = 'B'
        self.anchorC_msg.anchor_id = 'C'
        self.anchorA_msg.variance = 0.01
        self.anchorA_msg.variance = 0.01
        self.anchorA_msg.variance = 0.01
        # create anchor msg array (will store anchors A, B, C)
        self.anchor_array_msg = uwb_anchor_array()
        self.anchor_array_msg.header.frame_id = frame_id
        self.anchor_array_msg.tag_id = self.tag_id # the name of your tag: "front" for example
        self.anchor_array_msg.anchors = [None, None, None]
        # msg to inform user that node is initialized
        rospy.loginfo('uwb driver node initialized...')


    def start_uwb_driver(self):
        '''
        ultra wide band driver main loop
        '''
        # trigger board to start getting values
        self.tag.start_reading_acquisition()
        # give some time to the borad to get readings
        rospy.sleep(0.5)
        while not rospy.is_shutdown():
            # read anchor values and store in ros msg
            response = self.tag.read_anchors(debug=False)
            # check if response is valid
            if response == None:
                rospy.logwarn('No msg received')
                self.loop_rate.sleep()
                continue
            # fill timestamp
            self.anchor_array_msg.header.stamp = rospy.Time.now()
            # all ok, readings where received properly
            rospy.logdebug('all ok')
            # response has the form: [meanA, varA], [meanB, varB], [meanC, varC]
            # unpack values
            anchorA, anchorB, anchorC = response
            # set mean as the value for the radius
            self.anchorA_msg.radius = anchorA[0]
            self.anchorB_msg.radius = anchorB[0]
            self.anchorC_msg.radius = anchorC[0]
            # set variance value
            self.anchorA_msg.variance = anchorA[1]
            self.anchorB_msg.variance = anchorB[1]
            self.anchorC_msg.variance = anchorC[1]
            # pack anchors into anchor array msg
            self.anchor_array_msg.anchors[0] = self.anchorA_msg
            self.anchor_array_msg.anchors[1] = self.anchorB_msg
            self.anchor_array_msg.anchors[2] = self.anchorC_msg
            # publish anchor array msg
            self.uwb_pub.publish(self.anchor_array_msg)
            # sleep to control the node frequency
            self.loop_rate.sleep()
        # send serial port command to stop the readings flow
        self.tag.stop_reading_acquisition()
        # close serial port
        self.tag.close_port()


def main():
    # create and register node in the network
    rospy.init_node('uwb_driver_node', anonymous=False)
    # create object of UWBDriverNode class (constructor will get executed one time only)
    uwb_driver = UWBDriverNode()
    # call driver main loop function
    uwb_driver.start_uwb_driver()
