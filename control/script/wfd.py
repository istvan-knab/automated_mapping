#!/usr/bin/env python3

import sys
import time

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray, Marker

import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header

from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5


class Costmap2d:
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 253
        LethalObstacle = 254
        NoInformation = 255

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return self.map.info.width, self.map.info.height

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class OccupancyGrid2d:
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return self.map.info.width, self.map.info.height

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y:
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if my > self.map.info.height or mx > self.map.info.width:
            raise Exception("Out of bounds")

        return mx, my

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierCache:
    cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}


class FrontierPoint:
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y


def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length


def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return loc.mapX, loc.mapY

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return mx, my


def getFrontier(pose, costmap):
    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (
                        PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (
                                PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in
                       getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if 0 < x < costmap.getSizeX() and 0 < y < costmap.getSizeY():
                neighbors.append(fCache.getPoint(x, y))

    return neighbors


def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree


class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


class WaypointFollowerTest():
    def __init__(self):
        rospy.init_node('nav2_waypoint_tester', anonymous=True)
        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.lastWaypoint = None
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.costmap = None

        rospy.loginfo('Running Waypoint Test')

    def moveToFrontiers(self):
        frontiers = getFrontier(self.currentPose, self.costmap)

        if len(frontiers) == 0:
            rospy.loginfo('No More Frontiers')
            return

        location = None
        largestDist = 0
        for f in frontiers:
            dist = math.sqrt(((f[0] - self.currentPose.position.x) ** 2) + ((f[1] - self.currentPose.position.y) ** 2))
            if dist > largestDist:
                largestDist = dist
                location = [f]

        rospy.loginfo(f'World points {location}')
        self.setWaypoints(location)

        """
        goal = MoveBaseGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = location[0][0]
        goal.target_pose.pose.position.y = location[0][1]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo('Sending goal request...')
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = location[0][0]
        goal.pose.position.y = location[0][1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish()
        rospy.sleep()

        self.moveToFrontiers()

    def setInitialPose(self, pose):
        init_pose = PoseWithCovarianceStamped()
        init_pose.pose.pose.position.x = pose[0]
        init_pose.pose.pose.position.y = pose[1]
        init_pose.header.frame_id = 'map'
        self.currentPose = init_pose.pose.pose
        self.publishInitialPose(init_pose)
        time.sleep(5)

    def poseCallback(self, msg):
        rospy.loginfo('Received amcl_pose')
        self.currentPose = msg.pose.pose

    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self):
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

    def publishInitialPose(self, init_pose):
        self.initial_pose_pub.publish(init_pose)

    def shutdown(self):
        rospy.loginfo('Shutting down')

    def cancel_goal(self):
        self.action_client.cancel_goal()
        self.action_client.wait_for_result()

    def info_msg(self, msg):
        rospy.loginfo(msg)

    def warn_msg(self, msg):
        rospy.logwarn(msg)

    def error_msg(self, msg):
        rospy.logerr(msg)


def main():
    wps = [[-0.52, -0.54], [0.58, -0.55], [0.58, 0.52]]
    starting_pose = [-2.0, -0.5]

    test = WaypointFollowerTest()
    test.setWaypoints(wps)

    test.info_msg('Setting initial pose')
    test.setInitialPose(starting_pose)
    test.info_msg('Waiting for amcl_pose to be received')
    
    test.moveToFrontiers()

    rospy.spin()


if __name__ == '__main__':
    main()
