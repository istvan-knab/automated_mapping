#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class Path_Creator:
    def __init__(self):
        self.path_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1), self.callback_timer)

    def callback_timer(self, event):
        rospy.loginfo("Hello")
        



if __name__ == '__main__':
    rospy.init_node("random_control")
    path_publisher = Path_Creator()
    rospy.spin()