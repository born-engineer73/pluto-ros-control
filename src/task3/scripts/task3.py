#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32

def read_serial_data():
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust the port name as necessary
    pub = rospy.Publisher('deltaX', Int32, queue_size=10)
    rospy.init_node('serial_reader', anonymous=True)
    
    #rate = rospy.Rate(10)  # 10hz
   
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline()
            try:
                decoded_data = data.decode('utf-8').strip()
                deltaX = int(decoded_data)  # Assuming the sensor returns integer values
                rospy.loginfo("Publishing deltaX: {}".format(deltaX))
                pub.publish(deltaX)
            except UnicodeDecodeError:
                rospy.logwarn("Received raw bytes: {}".format(data))
            except ValueError:
                rospy.logwarn("Received non-integer data: {}".format(decoded_data))
        #rate.sleep()

if __name__ == '__main__':
    try:
        read_serial_data()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
