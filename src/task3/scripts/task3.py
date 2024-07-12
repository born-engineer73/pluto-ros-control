#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

def read_serial_data():
    ser = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust the port name as necessary
    pub = rospy.Publisher('deltaX', Int32MultiArray, queue_size=10)
    rospy.init_node('serial_reader', anonymous=True)
    
    #rate = rospy.Rate(10)  # 10hz
   
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline()
            try:
		int_array=Int32MultiArray()
                decoded_data = data.decode('utf-8').strip()
		data_array=decoded_data.split(",")
		deltaX = int(data_array[0])
		deltaY = int(data_array[1])
		int_array.data= [deltaX,deltaY]
                  # Assuming the sensor returns integer values
                rospy.loginfo("Publishing deltaX: {}".format(int_array.data))
                pub.publish(int_array)
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
