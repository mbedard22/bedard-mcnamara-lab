#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState



class Director:

    def __init__(self):
        self.pub = rospy.Publisher('/lane_controller_node/car_cmd',Twist2DStamped,queue_size=10)
        rospy.Subscriber('/fsm_node/mode', FSMState, self.mode_trigger)

        self.running = FALSE
        
        
    def mode_trigger(self,msg):
                  
        if msg.state == "LANE_FOLLOWING" and self.running == FALSE:
            self.running = TRUE
            # start driving
             
        else if msg.state == "LANE_FOLLOWING" and self.running == TRUE:
            break;
            
        else if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.running = FALSE
          
       
    def drive_straight(self, msg):
        if self.running == TRUE:
            cmd = Twist2DStamped()
            cmd.v = 2
            cmd.omega = 0
          
            rospy.pub.publish(cmd)
    def turn(self, msg):
        if self.running == TRUE:
            cmd = Twist2DStamped()
            cmd.v = 0
            cmd.omega = 5
            
            rospy.publish(cmd)
            
if __name__=='__main__':
    try:
        rospy.init_node('director',anonymous=True)
        d = Director()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for i in range[3]:
               
               d.drive_straight()
               rate.sleep()
               d.turn()
               rate.sleep()
               
    except rospy.ROSInterruptException:
        pass

