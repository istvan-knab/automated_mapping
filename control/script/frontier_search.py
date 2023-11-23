#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import costmap_2d
import math
from queue import Queue
from explore.costmap_tools import nhood4, nhood8

FREE_SPACE = 0
NO_INFORMATION = -1
LETHAL_OBSTACLE = 100


class Frontier:
    def __init__(self):
        self.centroid = Point()
        self.size = 0
        self.min_distance = float('inf')
        self.points = []
        self.initial = Point()
        self.middle = Point()
        self.cost = 0.0


class FrontierSearch:
    def __init__(self, costmap, potential_scale, gain_scale, min_frontier_size):
        self.costmap = costmap
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.min_frontier_size = min_frontier_size

    def search_from(self, position):
        frontier_list = []

        mx, my = self.costmap.worldToMap(position.x, position.y)
        if mx is None or my is None:
            rospy.logerr("Robot out of costmap bounds, cannot search for frontiers")
            return frontier_list

        with self.costmap.mutex:
            self.map_ = self.costmap.getCharMap()
            self.size_x_ = self.costmap.getSizeInCellsX()
            self.size_y_ = self.costmap.getSizeInCellsY()

        frontier_flag = [False] * (self.size_x_ * self.size_y_)
        visited_flag = [False] * (self.size_x_ * self.size_y_)

        bfs = Queue()

        clear, pos = self._nearest_cell(FREE_SPACE, mx, my)
        if clear is not None:
            bfs.put(clear)
        else:
            bfs.put(pos)
            rospy.logwarn("Could not find nearby clear cell to start search")

        visited_flag[bfs.queue[0]] = True

        while not bfs.empty():
            idx = bfs.get()

            for nbr in nhood4(idx, self.costmap):
                if self.map_[nbr] <= self.map_[idx] and not visited_flag[nbr]:
                    visited_flag[nbr] = True
                    bfs.put(nbr)
                elif self._is_new_frontier_cell(nbr, frontier_flag):
                    frontier_flag[nbr] = True
                    new_frontier = self._build_new_frontier(nbr, pos, frontier_flag)
                    if new_frontier.size * self.costmap.getResolution() >= self.min_frontier_size:
                        frontier_list.append(new_frontier)

        for frontier in frontier_list:
            frontier.cost = self._frontier_cost(frontier)

        frontier_list.sort(key=lambda f: f.cost)

        return frontier_list

    def _build_new_frontier(self, initial_cell, reference, frontier_flag):
        output = Frontier()
        output.centroid.x = 0
        output.centroid.y = 0
        output.size = 1
        output.min_distance = float('inf')

        ix, iy = self.costmap.indexToCells(initial_cell)
        self.costmap.mapToWorld(ix, iy, output.initial.x, output.initial.y)

        bfs = Queue()
        bfs.put(initial_cell)

        rx, ry = self.costmap.indexToCells(reference)
        reference_x, reference_y = self.costmap.mapToWorld(rx, ry)

        while not bfs.empty():
            idx = bfs.get()

            for nbr in nhood8(idx, self.costmap):
                if self._is_new_frontier_cell(nbr, frontier_flag):
                    frontier_flag[nbr] = True
                    mx, my = self.costmap.indexToCells(nbr)
                    wx, wy = self.costmap.mapToWorld(mx, my)

                    point = Point()
                    point.x = wx
                    point.y = wy
                    output.points.append(point)

                    output.size += 1

                    output.centroid.x += wx
                    output.centroid.y += wy

                    distance = math.sqrt((reference_x - wx) ** 2 + (reference_y - wy) ** 2)
                    if distance < output.min_distance:
                        output.min_distance = distance
                        output.middle.x = wx
                        output.middle.y = wy

                    bfs.put(nbr)

        output.centroid.x /= output.size
        output.centroid.y /= output.size
        return output

    def _is_new_frontier_cell(self, idx, frontier_flag):
        if self.map_[idx] != NO_INFORMATION or frontier_flag[idx]:
            return False

        for nbr in nhood4(idx, self.costmap):
            if self.map_[nbr] == FREE_SPACE:
                return True

        return False

    def _frontier_cost(self, frontier):
        return (self.potential_scale * frontier.min_distance * self.costmap.getResolution()) - \
               (self.gain_scale * frontier.size * self.costmap.getResolution())

    def _nearest_cell(self, target_cost, reference_x, reference_y):
        max_distance = 3.0 * self.costmap.getResolution()

        reference_index = self.costmap.getIndex(reference_x, reference_y)

        for distance in range(1, int(max_distance / self.costmap.getResolution()) + 1):
            cells = self.costmap.getCircle(reference_index, distance)

            for cell in cells:
                if self.map_[cell] == target_cost:
                    return cell, reference_index

        return None, None


if __name__ == "__main__":
    rospy.init_node("frontier_search_node")

    costmap = costmap_2d.Costmap2D()
    frontier_search = FrontierSearch(costmap, potential_scale=1.0, gain_scale=1.0, min_frontier_size=0.5)

    position = Point(x=0.0, y=0.0)  # Set the initial position of the robot

    frontiers = frontier_search.search_from(position)
    for i, frontier in enumerate(frontiers):
        rospy.loginfo(f"Frontier {i + 1}: Centroid: ({frontier.centroid.x}, {frontier.centroid.y}), "
                      f"Size: {frontier.size}, Min Distance: {frontier.min_distance}, Cost: {frontier.cost}")

    rospy.spin()
