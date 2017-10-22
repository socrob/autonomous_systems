#!/usr/bin/env python

'''
 * Copyright [2017] <Instituto Superior Tecnico>
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Rviz visualization for the open source driver for
 * the Ultra Wide Band sensor by Eliko:
 *
 *     https://www.eliko.ee/products/kio-rtls/
 *
'''

import rospy
from monarch_uwb.msg import uwb_anchor_array
from visualization_msgs.msg import Marker

class UWBVisualization(object):
    '''
    Listen to anchor readings (Ultra Wide Band sensor) and display marker array
    spheres, for visualization purposes
    '''
    def __init__(self):
        # get from param server the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # subscribe to anchor msg
        rospy.Subscriber("~tag_readings", uwb_anchor_array, self.anchorsCallBack, queue_size=1)
        # to publish marker visualization msg
        self.uwb_visualization_pub = rospy.Publisher('~marker_anchors', Marker, queue_size=1)
        # flag to indicate that an anchor msg was anchor_msg_received
        self.anchors_msg_received = False
        # to receive the anchors msg
        self.anchors_msg = None
        # msg to inform user that node is initialized
        rospy.loginfo('uwb driver visualization node initialized...')
        

    def anchorsCallBack(self, msg):
        '''
        will get executed everytime an anchor msg arrives
        saves the received data into a member variable and raises a flag
        '''
        self.anchors_msg_received = True
        self.anchors_msg = msg


    def create_sphere_marker(self, frame_id, radius, namespace, color):
        '''
        generic function that received a radius and a frame id and creates
        a sphere as marker for visualization purposes
        '''
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # marker namespace
        marker.ns = namespace
        marker.text = namespace

        # if radius is zero draw a huge sphere
        if radius < 0.001:
            radius = 5.0

        # marker scale
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius

        # marker color
        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # return the marker
        return marker

    def create_and_publish_markers(self):
        '''
        calls create_sphere_marker 3 times to
        publish the three anchors
        '''
        for i in range(3):
            # identify anchor to set color
            if self.anchors_msg.anchors[i].anchor_id == 'A':
                # color is transparency, r, g, b
                color = [0.7, 0.0, 0.0, 1.0]
            elif self.anchors_msg.anchors[i].anchor_id == 'B':
                color = [0.7, 0.0, 1.0, 0.0]
            else:
                color = [0.7, 0.0, 1.0, 1.0]
            # set red color is anchor is 0
            if self.anchors_msg.anchors[i].radius < 0.001:
                color = [0.2, 1.0, 0.0, 0.0]
            # create marker
            marker_msg = self.create_sphere_marker(
                self.anchors_msg.header.frame_id,
                self.anchors_msg.anchors[i].radius,
                self.anchors_msg.anchors[i].anchor_id,
                color)
            # publish marker msg
            self.uwb_visualization_pub.publish(marker_msg)


    def start_uwb_visualization_node(self):
        '''
        uwb main loop function
        '''
        while not rospy.is_shutdown():
            # check if anchors msg was received
            if self.anchors_msg_received:
                # lower flag
                self.anchors_msg_received = False
                # create and publish markers
                self.create_and_publish_markers()
            self.loop_rate.sleep()


def main():
    rospy.init_node('uwb_visualization_node', anonymous=False)
    ultra_wide_band_visualization = UWBVisualization()
    ultra_wide_band_visualization.start_uwb_visualization_node()
