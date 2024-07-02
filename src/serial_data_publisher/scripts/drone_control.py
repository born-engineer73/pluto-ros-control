#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32, Float64
from plutodrone.srv import PlutoPilot, PlutoPilotRequest, PlutoPilotResponse

class DroneController:
    def __init__(self):
        rospy.init_node('serial_data_publisher')

        # Publishers and Subscribers
        self.pub = rospy.Publisher('serial_data_topic', Int32, queue_size=10)
        rospy.Subscriber('current_pitch', Float64, self.pitch_callback)
        rospy.Subscriber('serial_data_topic', Int32, self.distance_callback)

        # Drone control variables
        self.original_pitch = 1400
        self.current_pitch = self.original_pitch
        self.distance = None

        # Serial port configuration
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
        rospy.loginfo("Connected to {} at {} baudrate.".format(self.port, self.baudrate))

        # Start reading from the serial port
        self.read_from_serial()

    def read_from_serial(self):
        while not rospy.is_shutdown():
            if self.serial_connection.in_waiting > 0:
                data = self.serial_connection.readline().strip()
                rospy.loginfo("Received raw data: {}".format(data))
                try:
                    self.distance = int(data.split(":")[-1].strip())
                    rospy.loginfo("Publishing distance data: {}".format(self.distance))
                    self.pub.publish(self.distance)
		    self.distance_callback(self.distance)
                except ValueError:
                    rospy.logwarn("Unexpected data format received.")

    def pitch_callback(self, msg):
        self.current_pitch = msg.data

    def distance_callback(self, msg):
        distance = msg
        adjusted_pitch = self.adjust_pitch(distance)
	rospy.loginfo(adjusted_pitch)
        self.publish_drone_command(adjusted_pitch)

    def adjust_pitch(self, distance):
        if distance < 10:
            return 1500
        elif distance < 100:
            # Linearly adjust pitch based on distance
            return self.original_pitch * (distance / 100.0)
        else:
            return self.original_pitch

    def publish_drone_command(self, pitch):
        command = PlutoPilotRequest()
        command.pitch = pitch
        rospy.wait_for_service('PlutoService')
	rospy.loginfo("Vaibhav")
        try:
            pluto_service = rospy.ServiceProxy('PlutoService', PlutoPilot)
            response = pluto_service(command)
            rospy.loginfo("Published drone command with pitch: {}".format(pitch))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

if __name__ == "__main__":
    try:
        DroneController()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
