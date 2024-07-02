#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32

def read_from_serial(port, baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo("Connected to {} at {} baudrate.".format(port, baudrate))

        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                data = ser.readline().strip()
                rospy.loginfo("Received raw data: {}".format(data))
                try:
                    data_int = int(data.split(":")[-1].strip())
                    rospy.loginfo("Publishing data: {}".format(data_int))
                    pub.publish(data_int)
                except ValueError:
                    rospy.logwarn("Unexpected data format received.")

    except serial.SerialException as e:
        rospy.logerr("SerialException: {}".format(e))

if __name__ == "__main__":
    rospy.init_node('serial_data_publisher')
    pub = rospy.Publisher('serial_data_topic', Int32, queue_size=10)

    # Replace '/dev/ttyUSB0' with your actual serial port name
    port = '/dev/ttyUSB0'
    baudrate = 115200

    read_from_serial(port, baudrate)
    rospy.spin()
