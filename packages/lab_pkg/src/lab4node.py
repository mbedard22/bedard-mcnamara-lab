#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import Twist2DStamped

class PID_controller:
    def __init__(self):
        rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)

        self.pub=rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)

        self.prev_error_A = 0
        self.cum_error_A = 0
        self.prev_error_D = 0
        self.cum_error_D = 0
        self.elapsedTime = 0.03 ##need to find actual timestep




    def callback(self, msg):
            cmd = Twist2DStamped()

            #PID for angular
            self.error_A = msg.detections[0].transform.translation.x

            K_p_a = 1.75
            K_i_a = 0.05
            K_d_a = 0.2

            #determine error between target and current position
            #need to determine elapsed time for integral and derivative 

            self.cum_error_A += self.error_A * self.elapsedTime
            rate_error_A = (self.error_A-self.prev_error_A)/self.elapsedTime

            cmd.omega = (K_p_a * self.error_A) + (K_i_a * self.cum_error_A) + (K_d_a * rate_error_A)
            cmd.omega = -cmd.omega
            self.pub.publish(cmd)
          #  rospy.logwarn(cmd.omega)

            self.prev_error_A = msg.detections[0].transform.translation.x

            cmd2 = Twist2DStamped()

            #PID for angular
            self.error_D = msg.detections[0].transform.translation.z

            K_p_d = .5
            K_i_d = 0.05
            K_d_d = 0.4

            #determine error between target and current position
            #need to determine elapsed time for integral and derivative 
            if(self.error_D < .15):
                cmd2.v = 0
                self.pub.publish(cmd2)

                rospy.logwarn(cmd2.v)

                self.prev_error_D = msg.detections[0].transform.translation.z
            else:
                self.cum_error_D += self.error_D * self.elapsedTime
                rate_error_D = (self.error_D-self.prev_error_D)/self.elapsedTime

                cmd2.v = (K_p_d * self.error_D) + (K_i_d * self.cum_error_D) + (K_d_d * rate_error_D)

                self.pub.publish(cmd2)
                rospy.logwarn(cmd2.v)

                self.prev_error_D = msg.detections[0].transform.translation.z

if __name__=='__main__':
    rospy.init_node("lab4node.py", anonymous = True)
    p = PID_controller()
    rospy.spin()



