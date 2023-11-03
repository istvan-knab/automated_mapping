#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan


class LaserRead:
    """
    This Node is responsible for reading lidar data and to prepare it to use by SLAM
    """
    def __init__(self):
        
        self.sub = rospy.Subscriber("/agent1/scan", LaserScan, self.scan_callback, queue_size=1)
        

    def scan_callback(self, scan_in : LaserScan):
        rospy.loginfo("Hello, Lidar is active")

if __name__ == '__main__':
    rospy.init_node("lidar_read")

    laser_state = LaserRead()
    
    rospy.spin()