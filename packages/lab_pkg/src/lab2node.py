#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState



class Director:

    def __init__(self):
        self.pub = rospy.Publisher('lane_controller_node/car_cmd',Twist2DStamped,queue_size=10)
        rospy.Subscriber('fsm_node/mode', FSMState, self.mode_trigger)

        self.running = False
        
        
    def mode_trigger(self,msg):
                  
        if msg.state == "LANE_FOLLOWING" and self.running == False:
            self.running = True
            # start driving
            
        elif msg.state == "LANE_FOLLOWING" and self.running == True:
            pass
            
        elif msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.running = False
          
       
    def drive_straight(self):
        if self.running == True:
            cmd = Twist2DStamped()
            cmd.v = 2
            cmd.omega = 0
          
            self.pub.publish(cmd)
            self.pub.publish(cmd)
    
    def turn(self):
        if self.running == True:
            cmd = Twist2DStamped()
            cmd.v = 0
            cmd.omega = 5
            
            self.pub.publish(cmd)
            self.pub.publish(cmd)
            
if __name__=='__main__':
    try:
        rospy.init_node('director',anonymous=True)
        d = Director()
        rate = rospy.Rate(100)
        
       
        while not rospy.is_shutdown():
            
            #for i in range(3):
               
            d.drive_straight()
            rate.sleep()
            d.turn()
            rate.sleep()
               
    except rospy.ROSInterruptException:
        pass

