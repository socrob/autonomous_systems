#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to laser scanner data
from sensor_msgs.msg import LaserScan

# to be able to publish Twist data (and move the robot)
from geometry_msgs.msg import Twist

class pioneerSimpleBehavior(object):
    '''
    Exposes a behavior for the pioneer robot so that moves forward until
    it has an obstacle at 1.0 m then stops rotates for some time to the
    right and resumes motion.
    '''
    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        # register node in ROS network
        rospy.init_node('pioneer_smart_behavior', anonymous=False)
        # print message in terminal
        rospy.loginfo('Pioneer simple behavior started !')
        # subscribe to pioneer laser scanner topic
        rospy.Subscriber("robot_0/base_scan_1", LaserScan, self.laserCallback)
        # setup publisher to later on move the pioneer base
        self.pub_cmd_vel = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        self.distance = 10.0

    def rotate_right_for_some_time(self):
        '''
        Rotate the robot for some time
        '''
        # create empty message of Twist type (check http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
        twist_msg = Twist()
        # liner speed
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = -0.3
        # base needs this msg to be published constantly for the robot to keep moving
        for i in range(1,20):
            # publish message
            self.pub_cmd_vel.publish(twist_msg)
            # sleep for a small amount of time
            rospy.sleep(0.1)


    def move_forward_until_distance_is_short(self):
        '''
        Move the robot forward until it detects an obstacle 20 cm in front of him
        then calls rotate_right_for_some_time
        '''
        # create empty message of Twist type (check http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
        twist_msg = Twist()
        # linear speed
        twist_msg.linear.x = 0.5
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        # base needs this msg to be published constantly for the robot to keep moving so we publish in a loop
        # while the distance from the robot to the walls is bigger than 1 m keep looping
        while self.distance > 1.0:
            # publish Twist message to /robot_0/cmd_vel to move the robot
            self.pub_cmd_vel.publish(twist_msg)
            # sleep for a small amount of time
            rospy.sleep(0.1)
        # distance was reported to be less than 1.0 m, lets rotate for some time (to avoid collision)
        self.rotate_right_for_some_time()


    def laserCallback(self, msg):
        '''
        This function gets executed everytime a laser scanner msg is received on the
        topic: /robot_0/base_scan_1
        '''
        # ============= YOUR CODE GOES HERE! =====
        # hint: is just one line of code
        # hint: msg contains the laser scanner msg
        # hint: check http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html



        # ============= YOUR CODE ENDS HERE! =====


    def run_behavior(self):
        while not rospy.is_shutdown():
            self.move_forward_until_distance_is_short()


if __name__ == '__main__':
    # create object of the class pioneerSimpleBehavior (constructor will get executed!)
    my_object = pioneerSimpleBehavior()
    # call run_behavior method of class pioneerSimpleBehavior
    my_object.run_behavior()
