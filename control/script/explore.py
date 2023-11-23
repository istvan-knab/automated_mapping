#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer', anonymous=True)

        # Subscribe to the map topic to get occupancy grid information
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Initialize a simple goal publisher for testing
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Initialize a simple action client for MoveBase
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.timer_goal = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        # Initialize the occupancy grid
        self.occupancy_grid = None
        self.global_x = 0.0
        self.global_y = 0.0

    def timer_callback(self, event):
        self.publish_goal(self.global_x, self.global_y)

    def map_callback(self, msg):
        self.occupancy_grid = msg

    def explore_frontier(self):
        # Check if the occupancy grid is available
        if self.occupancy_grid is None:
            rospy.logwarn("Occupancy grid not available yet.")
            return

        # Assume that the origin of the map is at (0, 0)
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        # Iterate through the occupancy grid to find frontiers
        for i in range(len(self.occupancy_grid.data)):
            if self.occupancy_grid.data[i] == 0:  # Unexplored cell (0)
                # Convert the 1D index to 2D coordinates
                x = i % self.occupancy_grid.info.width
                y = i // self.occupancy_grid.info.width

                # Convert map coordinates to global coordinates
                self.global_x = origin_x + x * self.occupancy_grid.info.resolution
                self.global_y = origin_y + y * self.occupancy_grid.info.resolution

                # Publish a simple goal for testing (you may replace this with your exploration logic)
                

    def publish_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

    def run(self):
        rate = rospy.Rate(10)  # 1 Hz

        while not rospy.is_shutdown():
            self.explore_frontier()
            rate.sleep()

if __name__ == '__main__':
    try:
        frontier_explorer = FrontierExplorer()
        frontier_explorer.run()
    except rospy.ROSInterruptException:
        pass