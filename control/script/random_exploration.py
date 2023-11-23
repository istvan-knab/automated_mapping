#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from rospy import Time

class Path_Creator:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.path_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(5.0), self.timer_callback)
        rospy.loginfo("Starting random exploring Node")
        rospy.logwarn("Timer example node is running.")

    def timer_callback(self, event):
        
        rospy.logwarn("Hello")
        closest_point = PoseStamped()
        closest_point.header.frame_id = "map"
        closest_point.header.stamp = rospy.Time.now()
        closest_point.pose.position.x = random.randint(-1,1) * random.random()
        closest_point.pose.position.y = random.randint(-1,1) * random.random()
        closest_point.pose.position.z = 0.0

        # A quaternion nem lehet (0,0,0,0) értékű
        closest_point.pose.orientation.w = random.random()

        closest_point_map = PoseStamped()
        if self.tfBuffer.can_transform("map", closest_point.header.frame_id, closest_point.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",closest_point.header.frame_id, closest_point.header.stamp)

            closest_point_map = tf2_geometry_msgs.do_transform_pose(closest_point, trans_base2map)

        self.path_pub.publish(closest_point_map)
    

if __name__ == '__main__':
    rospy.init_node("random_control")
    path_publisher = Path_Creator()

    rospy.spin()