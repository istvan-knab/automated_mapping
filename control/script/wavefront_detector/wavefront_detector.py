#!/usr/bin/env python3

import time
import math
from typing import List, Tuple, Optional

import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry

from custom_occupancy_grid_2d import CustomOccupancyGrid2D
import global_functions


class WaveFrontDetector:
    """
    Class representing a wavefront detector for navigation.

    Attributes:
        waypoints (Optional[List[PoseStamped]]): List of waypoints to navigate to.
        ready_to_move (bool): Flag indicating if the robot is ready to move.
        current_pose (Optional[PoseStamped]): Current pose of the robot.
        last_waypoint (Optional[PoseStamped]): Last waypoint visited by the robot.
        action_client (actionlib.SimpleActionClient): Action client for the move_base action server.
        initial_pose_pub (rospy.Publisher): Publisher for the initial pose.
        goal_pub (rospy.Publisher): Publisher for the goal pose.
        map_sub (rospy.Subscriber): Subscriber for the occupancy grid map.
        occupancy_grid_map (Optional[CustomOccupancyGrid2D]): Custom occupancy grid representing the costmap.
    """

    def __init__(self):
        """
        Initializes the WaveFrontDetector class with ROS components: publishers, subscribers, and an action client.
        """

        rospy.init_node('wavefront_detector', anonymous=True)
        self.waypoints: Optional[List[PoseStamped]] = None
        self.ready_to_move: bool = True
        self.current_pose: Optional[PoseStamped] = None
        self.last_waypoint: Optional[PoseStamped] = None
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.costmap_callback)

        self.occupancy_grid_map = None

        rospy.loginfo('Running Waypoint Test')

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        """
        Callback function for the occupancy grid map.

        Args:
            msg (OccupancyGrid): The received occupancy grid map.
        """
        self.occupancy_grid_map = CustomOccupancyGrid2D(msg)

    def move_to_frontiers(self) -> None:
        """
        Move the robot to identified frontiers in the environment.
        """

        frontiers = global_functions.get_frontiers(self.current_pose, self.occupancy_grid_map)

        if not frontiers:
            rospy.loginfo('No More Frontiers')
            return

        location = max(frontiers, key=lambda f: math.sqrt(((f[0] - self.current_pose.position.x) ** 2) +
                                                          ((f[1] - self.current_pose.position.y) ** 2)))

        rospy.loginfo(f'World points {location}')
        self.set_waypoints([location])

        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location[0]
        goal.target_pose.pose.position.y = location[1]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo('Sending goal request...')
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()

        self.move_to_frontiers()

    def set_initial_pose(self, pose: List[float]) -> None:
        """
        Set the initial pose of the robot.

        Args:
            pose (List[float]): The initial pose coordinates [x, y].
        """
        init_pose = PoseWithCovarianceStamped()
        init_pose.pose.pose.position.x = pose[0]
        init_pose.pose.pose.position.y = pose[1]
        init_pose.header.frame_id = 'map'
        self.current_pose = init_pose.pose.pose
        self.publish_initial_pose(init_pose)
        time.sleep(5)

    def pose_callback(self, msg: Odometry) -> None:
        """
        Callback function for the robot's pose.

        Args:
            msg (Odometry): The received odometry message.
        """

        rospy.loginfo('Received pose')
        self.current_pose = PoseStamped(pose=msg.pose.pose, header=msg.header)

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """
        Set the waypoints for the robot.

        Args:
            waypoints (List[Tuple[float, float]]): List of waypoint coordinates [(x1, y1), (x2, y2), ...].
        """

        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self) -> bool:
        """
        Returns:
            bool: True if the navigation was successful, False otherwise.
        """
        if not self.waypoints:
            rospy.error('Did not set valid waypoints before running test!')
            return False

        rospy.loginfo("'move_base' action server waiting...")
        self.action_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        for wp in self.waypoints:
            goal.target_pose.pose = wp.pose
            rospy.loginfo('Sending goal request...')
            self.action_client.send_goal(goal)
            self.action_client.wait_for_result()

        return True

    def publish_initial_pose(self, init_pose: PoseWithCovarianceStamped) -> None:
        """
        Publish the initial pose of the robot.

        Args:
            init_pose (PoseWithCovarianceStamped): The initial pose message.
        """

        self.initial_pose_pub.publish(init_pose)

    @staticmethod
    def shutdown() -> None:
        """
        Shutdown function to handle ROS node shutdown.
        """

        rospy.loginfo('Shutting down')

    def cancel_goal(self) -> None:
        """
        Cancel the current goal.
        """

        self.action_client.cancel_goal()
        self.action_client.wait_for_result()

    @staticmethod
    def info_msg(msg: str) -> None:
        """
        Log an information message.

        Args:
            msg (str): The information message.
        """

        rospy.loginfo(msg)

    @staticmethod
    def warn_msg(msg: str) -> None:
        """
        Log a warning message.

        Args:
            msg (str): The warning message.
        """

        rospy.logwarn(msg)

    @staticmethod
    def error_msg(msg: str) -> None:
        """
        Log an error message.

        Args:
            msg (str): The error message.
        """

        rospy.logerr(msg)
