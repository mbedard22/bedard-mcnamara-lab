 #!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import Twist2DStamped

class PID_controller:
    def __init__(self):
        rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)
        rospy.Subscriber('fsm_node/mode', FSMState, self.mode_trigger)

        self.pub=rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size = 10)
        
        self.running = False
        
        
    def mode_trigger(self,msg):
                  
        if msg.state == "LANE_FOLLOWING" and self.running == False:
            self.running = True
            # start driving
            
        elif msg.state == "LANE_FOLLOWING" and self.running == True:
            pass
            
        elif msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.running = False
        

    def callback(self, msg):
        if self.running == True:
            cmd = Twist2DStamped()
            
            #PID for angular
            
            cmd.omega = msg.detections[0].transform.translation.x
            self.pub.publish(cmd)

if __name__=='__main__':
    rospy.init_node("lab4node.py", anonymous = True)
    p = PID_controller()
    rospy.spin()

