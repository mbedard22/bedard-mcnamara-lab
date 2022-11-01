#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import



class Director:

    def __init__(self):
        self.pub = rospy.Publisher('',Twist,queue_size=10)
        self.count = 0
        self.loop = 0
        
    def direct(self):
        cmd = Twist()
        cmd.linear.x=0
        cmd.linear.y=0
        cmd.linear.z=0
        cmd.angular.x=0
        cmd.angular.y=0
        cmd.angular.z=0
        self.count += 1
        if self.count < 20:
            cmd.linear.x = 1
            cmd.angular.z=0
            self.pub.publish(cmd)
            
        elif self.count < 30:
            cmd.linear.x = 0 
            cmd.angular.z = 1.5
            self.pub.publish(cmd)
        else:
            self.count = 0
            self.loop += 1
            cmd.angular.z = 0
            self.pub.publish(cmd)
            
        if self.loop > 200:
            rospy.spin()
            
            
        

if __name__=='__main__':
    try:
        rospy.init_node('director',anonymous=True)
        d = Director()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            d.direct()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    
    
    vehiclePose
