#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import SimpleClientGoalState
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseGoal, MoveBaseResult, MoveBaseAction
from tf import TransformListener
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from frontier_exploration.srv import GetNextFrontier, GetNextFrontierRequest
from frontier_exploration.msg import Frontier

from automated_mapping.control.script.frontier_search import FrontierSearch


class Explore:
    def __init__(self):
        self.private_nh = rospy.get_param("~")
        self.tf_listener = TransformListener(rospy.Duration(10.0))
        self.costmap_client = CostmapClient(self.tf_listener)
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.prev_distance = 0
        self.last_markers_count = 0

        self.planner_frequency = 1.0
        self.progress_timeout = rospy.Duration(30.0)
        self.visualize = False
        self.potential_scale = 1e-3
        self.orientation_scale = 0.0
        self.gain_scale = 1.0
        self.min_frontier_size = 0.5

        self.search = FrontierSearch(self.costmap_client.get_costmap(),
                                     self.potential_scale, self.gain_scale,
                                     self.min_frontier_size)

        if self.visualize:
            self.marker_array_publisher = rospy.Publisher("frontiers", MarkerArray, queue_size=10)

        rospy.loginfo("Waiting to connect to move_base server")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        self.exploring_timer = rospy.Timer(rospy.Duration(1.0 / self.planner_frequency),
                                           self.make_plan)

    def visualize_frontiers(self, frontiers):
        blue = ColorRGBA()
        blue.r = 0
        blue.g = 0
        blue.b = 1.0
        blue.a = 1.0
        red = ColorRGBA()
        red.r = 1.0
        red.g = 0
        red.b = 0
        red.a = 1.0
        green = ColorRGBA()
        green.r = 0
        green.g = 1.0
        green.b = 0
        green.a = 1.0

        rospy.logdebug("Visualizing %d frontiers", len(frontiers))
        markers_msg = MarkerArray()
        markers = markers_msg.markers
        m = Marker()

        m.header.frame_id = self.costmap_client.get_global_frame_id()
        m.header.stamp = rospy.Time.now()
        m.ns = "frontiers"
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 0
        m.color.g = 0
        m.color.b = 255
        m.color.a = 255
        m.lifetime = rospy.Duration(0)
        m.frame_locked = True

        min_cost = frontiers[0].cost if frontiers else 0.0

        m.action = Marker.ADD
        id = 0
        for frontier in frontiers:
            m.type = Marker.POINTS
            m.id = id
            m.pose.position = Point()
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.points = frontier.points
            if self.goal_on_blacklist(frontier.centroid):
                m.color = red
            else:
                m.color = blue
            markers.append(m)
            id += 1
            m.type = Marker.SPHERE
            m.id = id
            m.pose.position = frontier.initial
            scale = min(abs(min_cost * 0.4 / frontier.cost), 0.5)
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.points = []
            m.color = green
            markers.append(m)
            id += 1

        current_markers_count = len(markers)

        m.action = Marker.DELETE
        for i in range(id, self.last_markers_count):
            m.id = i
            markers.append(m)

        self.last_markers_count = current_markers_count
        self.marker_array_publisher.publish(markers_msg)

    def make_plan(self):
        pose = self.costmap_client.get_robot_pose()
        frontiers = self.search.search_from(pose.position)
        rospy.logdebug("Found %d frontiers", len(frontiers))

        if not frontiers:
            self.stop()
            return

        if self.visualize:
            self.visualize_frontiers(frontiers)

        frontier = next((f for f in frontiers if not self.goal_on_blacklist(f.centroid)), None)
        if not frontier:
            self.stop()
            return

        target_position = frontier.centroid

        same_goal = self.prev_goal == target_position
        self.prev_goal = target_position
        if not same_goal or self.prev_distance > frontier.min_distance:
            self.last_progress = rospy.Time.now()
            self.prev_distance = frontier.min_distance

        if rospy.Time.now() - self.last_progress > self.progress_timeout:
            self.frontier_blacklist.append(target_position)
            rospy.logdebug("Adding current goal to blacklist")
            self.make_plan()
            return

        if same_goal:
            return

        goal = MoveBaseGoal()
        goal.target_pose.pose.position = target_position
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.header.frame_id = self.costmap_client.get_global_frame_id()
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(goal,
                                        done_cb=lambda status, result: self.reached_goal(status, result, target_position))

    def goal_on_blacklist(self, goal):
        tolerance = 5
        costmap_2d = self.costmap_client.get_costmap()
        for frontier_goal in self.frontier_blacklist:
            x_diff = abs(goal.x - frontier_goal.x)
            y_diff = abs(goal.y - frontier_goal.y)

            if x_diff < tolerance * costmap_2d.get_resolution() and \
                    y_diff < tolerance * costmap_2d.get_resolution():
                return True
        return False

    def reached_goal(self, status, result, frontier_goal):
        rospy.logdebug("Reached goal with status: %s", status)
        if status == SimpleClientGoalState.ABORTED:
            self.frontier_blacklist.append(frontier_goal)
            rospy.logdebug("Adding current goal to blacklist")

        rospy.Timer(rospy.Duration(0), lambda event: self.make_plan(), True)

    def start(self):
        self.exploring_timer.start()

    def stop(self):
        self.move_base_client.cancel_all_goals()
        self.exploring_timer.shutdown()
        rospy.loginfo("Exploration stopped.")


if __name__ == "__main__":
    rospy.init_node("explore")

    explore = Explore()
    explore.start()

    rospy.spin()
