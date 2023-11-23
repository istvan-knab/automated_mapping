#!/usr/bin/env python3

import rospy


class NavigationNode:
    
    def __init__(self):

        rospy.loginfo("Starting frontier exploring Node")

        #get robot position
        self.position_timer = rospy.Timer(rospy.Duration(1), self.position_callback)
        #get position of frontiers
        self.timer_frontier = rospy.Timer(rospy.Duration(5), self.frontier_timer_callback)
        #publish it with markers
    
    def frontier_timer_callback(self, event = None):
        rospy.loginfo("Frontier callback is alive")
    
    def position_callback(self, event = None):
        rospy.loginfo("Position callback is alive")
    


if __name__ == "__main__":

    rospy.init_node("frontier_exploration_python")
    rospy.loginfo("Start Node")
    node = NavigationNode()
    rospy.spin()