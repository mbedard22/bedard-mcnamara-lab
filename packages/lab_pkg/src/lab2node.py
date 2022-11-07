#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState



class Director:

    def __init__(self):
        self.pub = rospy.Publisher('/lane_controller_node/car_cmd',Twist2DStamped,queue_size=10)
        rospy.Subscriber('/fsm_node/mode', FSMState, self.mode_trigger)
        #self.count = 0
        #self.loop = 0
    
        self.start_flag = "NORMAL_JOYSTICK_CONTROL"
        self.running = FALSE
        
        
     #rethink this section    
    def mode_trigger(self,msg):
        
       if self.running == FALSE:
          
          if msg.state == "LANE_FOLLOWING":
             
             self.start_flag = "LANE_FOLLOWING"
             self.running = TRUE
             # start driving
             
          else:
             start.running = FALSE
             self.start_flag = msg.state
       else:
          # do nothing
          
       
    def start_driving(self, msg):
       
       #if mode = self derving -> start driving
       #check for debounce
       
       if self.start_flag == FALSE:
          self.start_flag == TRUE
          #move
          
          cmd = Twist2DStamped()
          cmd.v = 2
          cmd.omega = 0
          
          rospy.pub.publish(cmd)
    
    """    
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
            
        if self.loop > 4:
            rospy.spin()
            
            
        """
      def turn(self):
      
      
      
            
        

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
    except rospy.ROSInterruptException:
        pass
    
    
    vehiclePose
    
    """
    
    need to listen to duckMachine/car_cmd_switch_node
    
    
    """
