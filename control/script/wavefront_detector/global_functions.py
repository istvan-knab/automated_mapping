#!/usr/bin/env python3

from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from frontier_cache import FrontierCache
from frontier_point import FrontierPoint
from custom_occupancy_grid_2d import CustomOccupancyGrid2D
from point_classification import PointClassification
import global_constants


def calculate_centroid(coords: List[Tuple[float, float]]) -> Tuple[float, float]:
    """
    Calculate the centroid of a list of coordinates.

    Args:
        coords (List[Tuple[float, float]]): List of coordinates.

    Returns:
        Tuple[float, float]: The centroid coordinates.
    """

    coords_array = np.array(coords)
    length = coords_array.shape[0]
    sum_x = np.sum(coords_array[:, 0])
    sum_y = np.sum(coords_array[:, 1])
    return sum_x / length, sum_y / length


def find_free_point(map_x: int, map_y: int, occupancy_grid: CustomOccupancyGrid2D,
                    frontier_cache: FrontierCache) -> Tuple[int, int]:
    """
    Find a free point in the vicinity of the given map coordinates.

    Args:
        map_x (int): The x-coordinate on the map.
        map_y (int): The y-coordinate on the map.
        occupancy_grid (CustomOccupancyGrid2D): The custom occupancy grid.
        frontier_cache (FrontierCache): The frontier cache.

    Returns:
        Tuple[int, int]: The coordinates of a free point.
    """

    bfs = [frontier_cache.get_frontier_point(map_x, map_y)]

    while bfs:
        loc = bfs.pop(0)

        if occupancy_grid.get_cost_at(loc.map_x, loc.map_y) == CustomOccupancyGrid2D.CostValues.FreeSpace.value:
            return loc.map_x, loc.map_y

        for n in get_neighbors(loc, occupancy_grid, frontier_cache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification |= PointClassification.MapClosed.value
                bfs.append(n)

    return map_x, map_y


def get_frontiers(current_pose: PoseStamped, occupancy_grid: CustomOccupancyGrid2D) -> List[Tuple[float, float]]:
    """
    Identify frontiers in the occupancy grid based on the current pose.

    Args:
        current_pose (PoseStamped): The current pose of the agent.
        occupancy_grid (CustomOccupancyGrid2D): The custom occupancy grid.

    Returns:
        List[Tuple[float, float]]: List of frontier centroid coordinates.
    """

    frontier_cache = FrontierCache()
    frontier_cache.clear_cache()

    map_x, map_y = occupancy_grid.world_to_map(current_pose.pose.position.x, current_pose.pose.position.y)

    free_point = find_free_point(map_x, map_y, occupancy_grid, frontier_cache)
    start = frontier_cache.get_frontier_point(free_point[0], free_point[1])
    start.classification = PointClassification.MapOpen.value
    map_point_queue = [start]

    frontiers = []

    while map_point_queue:
        p = map_point_queue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if is_frontier_point(p, occupancy_grid, frontier_cache):
            p.classification |= PointClassification.FrontierOpen.value
            frontier_queue = [p]
            new_frontier = []

            while frontier_queue:
                q = frontier_queue.pop(0)

                if q.classification & (
                        PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if is_frontier_point(q, occupancy_grid, frontier_cache):
                    new_frontier.append(q)

                    for w in get_neighbors(q, occupancy_grid, frontier_cache):
                        if w.classification & (PointClassification.FrontierOpen.value |
                                               PointClassification.FrontierClosed.value |
                                               PointClassification.MapClosed.value) == 0:
                            w.classification |= PointClassification.FrontierOpen.value
                            frontier_queue.append(w)

                q.classification |= PointClassification.FrontierClosed.value

            new_frontier_coords = [occupancy_grid.map_to_world(x.map_x, x.map_y) for x in new_frontier]
            if len(new_frontier) > global_constants.MIN_FRONTIER_SIZE:
                frontiers.append(calculate_centroid(new_frontier_coords))

        for v in get_neighbors(p, occupancy_grid, frontier_cache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(occupancy_grid.get_cost_at(x.map_x, x.map_y) == CustomOccupancyGrid2D.CostValues.FreeSpace.value
                       for x in get_neighbors(v, occupancy_grid, frontier_cache)):
                    v.classification |= PointClassification.MapOpen.value
                    map_point_queue.append(v)

        p.classification |= PointClassification.MapClosed.value

    return frontiers


def get_neighbors(point: FrontierPoint, occupancy_grid: CustomOccupancyGrid2D,
                  frontier_cache: FrontierCache) -> List[FrontierPoint]:
    """
    Get neighboring points of a given frontier point.

    Args:
        point (FrontierPoint): The frontier point.
        occupancy_grid (CustomOccupancyGrid2D): The custom occupancy grid.
        frontier_cache (FrontierCache): The frontier cache.

    Returns:
        List[FrontierPoint]: List of neighboring frontier points.
    """

    neighbors = []

    for x in range(point.map_x - 1, point.map_x + 2):
        for y in range(point.map_y - 1, point.map_y + 2):
            if 0 < x < occupancy_grid.get_size_x() and 0 < y < occupancy_grid.get_size_y():
                neighbors.append(frontier_cache.get_frontier_point(x, y))

    return neighbors


def is_frontier_point(point: FrontierPoint, occupancy_grid: CustomOccupancyGrid2D,
                      frontier_cache: FrontierCache) -> bool:
    """
    Check if a point is a frontier point.

    Args:
        point (FrontierPoint): The frontier point.
        occupancy_grid (CustomOccupancyGrid2D): The custom occupancy grid.
        frontier_cache (FrontierCache): The frontier cache.

    Returns:
        bool: True if the point is a frontier point, False otherwise.
    """

    if occupancy_grid.get_cost_at(point.map_x, point.map_y) != CustomOccupancyGrid2D.CostValues.NoInformation.value:
        return False

    has_free = any(occupancy_grid.get_cost_at(n.map_x, n.map_y) == CustomOccupancyGrid2D.CostValues.FreeSpace.value
                   for n in get_neighbors(point, occupancy_grid, frontier_cache))

    return has_free
