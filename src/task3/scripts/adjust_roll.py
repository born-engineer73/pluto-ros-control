#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int32
import rospy
import sys

class DroneController():
    def __init__(self):
        rospy.init_node('drone_controller')
        self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
        self.delta_x_sub = rospy.Subscriber('/deltaX', Int32, self.update_roll)

        self.cmd = PlutoMsg()
        self.cmd.rcRoll = 0
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.cmd.commandType = 0
        self.cmd.trim_roll = 0
        self.cmd.trim_pitch = 0
        self.cmd.isAutoPilotOn = 0

        self.rate = rospy.Rate(10)  # 10 Hz

    def update_roll(self, msg):
        delta_x = msg.data
        # Adjust roll based on deltaX value
        
       
	


	if delta_x > 20:
            self.cmd.rcRoll -= ((delta_x - 20)/20)
        elif delta_x < -20:
            self.cmd.rcRoll += ((20 - delta_x)/20)
	else:
	    self.cmd.rcRoll=0
        # Ensure roll stays within the valid range (e.g., 1000 to 2000)
        self.cmd.rcRoll = max(-100, min(100, self.cmd.rcRoll))

        rospy.loginfo("Adjusted roll: {}".format(self.cmd.rcRoll))
        self.command_pub.publish(self.cmd)

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.command_pub.publish(self.cmd)
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down drone controller")
        finally:
            self.reset()

    def reset(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1000
        self.command_pub.publish(self.cmd)
        rospy.loginfo("Resetting drone parameters to default")
        rospy.sleep(1)

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
