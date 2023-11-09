#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan

class Path_Creator:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.path_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped, queue_size=1)
        #self.timer = rospy.Timer(rospy.Duration(1), self.callback_timer)
        rospy.loginfo("Starting random exploring Node")
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

    def callback_timer(self, event):
        
        msg = LaserScan()
        closest_point = PoseStamped()
        closest_point.header = msg.header
        closest_point.pose.position.x = 10.0
        closest_point.pose.position.y = 10.0
        closest_point.pose.position.z = 0.0

        # A quaternion nem lehet (0,0,0,0) értékű
        closest_point.pose.orientation.w = 1

        closest_point_map = PoseStamped()
        if self.tfBuffer.can_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp)

            closest_point_map = tf2_geometry_msgs.do_transform_pose(closest_point, trans_base2map)

        self.path_pub.publish(closest_point_map)
    
    def scan_callback(self, msg: LaserScan):

        
        closest_point = PoseStamped()
        closest_point.header = msg.header
        closest_point.pose.position.x = -0.1
        closest_point.pose.position.y = 0.0
        closest_point.pose.position.z = 0.0

        # A quaternion nem lehet (0,0,0,0) értékű
        closest_point.pose.orientation.w = 0.01

        closest_point_map = PoseStamped()
        if self.tfBuffer.can_transform("map", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",msg.header.frame_id, msg.header.stamp)

            closest_point_map = tf2_geometry_msgs.do_transform_pose(closest_point, trans_base2map)

        self.path_pub.publish(closest_point_map)

    
        



if __name__ == '__main__':
    rospy.init_node("random_control")
    path_publisher = Path_Creator()
    rospy.spin()