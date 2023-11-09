#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class Path_Creator:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.path_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1), self.callback_timer)
        rospy.loginfo("Starting random exploring Node")

    def callback_timer(self, event):
        
        pose = PoseStamped()
        pose.pose.position.x = 10.0
        pose.pose.position.y = 10.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        if self.tfBuffer.can_transform("map", pose.header.frame_id, pose.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",pose.header.frame_id, pose.header.stamp)

            closest_point_map = tf2_geometry_msgs.do_transform_pose(pose, trans_base2map)
        
        self.path_pub.publish(pose)
        



if __name__ == '__main__':
    rospy.init_node("random_control")
    path_publisher = Path_Creator()
    rospy.spin()