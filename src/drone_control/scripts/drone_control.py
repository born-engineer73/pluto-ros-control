#!/usr/bin/env python
import rospy
from plutodrone.srv import *
from std_msgs.msg import Float64, Int32
import csv
import time

class RequestData:
    """docstring for RequestData"""
    def __init__(self):
        rospy.init_node('drone_board_data')
        self.start_time = time.time()
        self.data_file = open('drone_data.csv', 'wb')  # 'wb' mode for Python 2
        self.csv_writer = csv.writer(self.data_file)
        self.csv_writer.writerow(['accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ',
                                  'magX', 'magY', 'magZ', 'roll', 'pitch', 'yaw',
                                  'altitude', 'battery', 'power_consumed'])
        rospy.Service('PlutoService', PlutoPilot, self.access_data)

    def access_data(self, req):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        if elapsed_time >= 10:
            rospy.signal_shutdown('10 seconds elapsed, shutting down')
       
        data_row = [req.accX, req.accY, req.accZ, req.gyroX, req.gyroY, req.gyroZ,
                    req.magX, req.magY, req.magZ, req.roll, req.pitch, req.yaw,
                    req.alt, req.battery, req.rssi]
       
        self.csv_writer.writerow(data_row)
        rospy.sleep(.1)
        
        print("1")
        return PlutoPilotResponse(rcAUX2=1500)

    def __del__(self):
        self.data_file.close()

if __name__ == '__main__':
    request_data = RequestData()
    rospy.spin()

