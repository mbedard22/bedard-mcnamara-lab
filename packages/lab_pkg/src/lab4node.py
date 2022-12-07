 #!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray
from duckietown_msgs.msg import Twist2DStamped

class PID_controller:
    def __init__(self):
        rospy.Subscriber("apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)
        self.pub=rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size = 10)

    def callback(self, msg):
        cmd = Twist2DStamped()
        cmd.v = msg.detections[0].transform.translation.x
        self.pub.publish(cmd)

if __name__=='__main__':
    rospy.init_node("lab4node.py", anonymous = True)
    p = PID_controller()
    rospy.spin()

