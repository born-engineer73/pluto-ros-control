#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from plutodrone.srv import *

class DroneStabilizer:
    def __init__(self):
        rospy.init_node('drone_stabilizer', anonymous=True)
        self.roll = 1500
        self.delta_x_sub = rospy.Subscriber('deltaX', Int32, self.delta_x_callback)
        self.service = rospy.ServiceProxy('PlutoService', PlutoPilot)
        rospy.wait_for_service('PlutoService')
        self.rate = rospy.Rate(10)  # 10hz

    def delta_x_callback(self, msg):
        delta_x = msg.data
        # Adjust roll based on deltaX value
        if delta_x > 10:
            self.roll -= (delta_x - 10)
        elif delta_x < -10:
            self.roll += (10 - delta_x)
        # Ensure roll stays within the valid range (e.g., 1000 to 2000)
        self.roll = max(1000, min(2000, self.roll))
        self.send_command_to_drone()

    def send_command_to_drone(self):
        try:
            response = self.service(roll=self.roll, pitch=1500, yaw=1500, 
                                    accX=1500, accY=1500, accZ=1500)
            rospy.loginfo("Sent roll command: {}".format(self.roll))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down drone stabilizer")

if __name__ == '__main__':
    stabilizer = DroneStabilizer()
    stabilizer.run()
