#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

class MyNode(object):
    def __init__(self):
        rospy.init_node("hello_node")
        
        self.hello_sub = rospy.Subscriber("hello", String, self._hello_cb)
        
        self._cmd_vel_pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=1)
        
        self.hello_received = False
        
        self.rate = rospy.Rate(10)
        
    def run(self):
        while not rospy.is_shutdown():
            if self.hello_received == True:
                rospy.loginfo("Hello received.")
                velocity = 0.0
                if rospy.has_param("speed"):
                    velocity = int(rospy.get_param("speed"))
                else:
                    rospy.logerr("Param does not exist")
                    return
                velocity = Vector3(x=velocity, y=0.0, z=0.0)
                movement = Twist(linear=velocity)
                self._cmd_vel_pub.publish(movement)
                
                self.hello_received = False
            
            self.rate.sleep()
            
    def _hello_cb(self, msg):
        if msg.data == 'hello':
            self.hello_received = True
            
            
my_obj = MyNode()
my_obj.run()