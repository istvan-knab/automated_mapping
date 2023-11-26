#!/usr/bin/env python3

from enum import Enum
from typing import Tuple

from nav_msgs.msg import OccupancyGrid, Odometry


class CustomOccupancyGrid2D:
    """
    Class representing a custom 2D occupancy grid with utility functions.
    """

    class CostValues(Enum):
        """
        Enumeration representing different cost values for the occupancy grid.

        Attributes:
            FreeSpace: Cost value for free space on the grid.
            InscribedInflated: Cost value for inscribed inflated space.
            LethalObstacle: Cost value for lethal obstacles on the grid.
            NoInformation: Cost value for cells with no information.
        """

        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, occupancy_grid: OccupancyGrid):
        """
        Initialize the CustomOccupancyGrid2D with an OccupancyGrid message.

        Args:
            occupancy_grid (OccupancyGrid): The ROS OccupancyGrid message.
        """

        self.occupancy_grid = occupancy_grid

    def get_cost_at(self, map_x: int, map_y: int) -> int:
        """
        Get the cost value at the specified map coordinates.

        Args:
            map_x (int): The x-coordinate on the map.
            map_y (int): The y-coordinate on the map.

        Returns:
            int: The cost value at the specified map coordinates.
        """

        return self.occupancy_grid.data[self.__get_index(map_x, map_y)]

    def get_size(self) -> Tuple[int, int]:
        """
        Get the size (width, height) of the occupancy grid.

        Returns:
            Tuple[int, int]: The width and height of the occupancy grid.
        """

        return self.occupancy_grid.info.width, self.occupancy_grid.info.height

    def get_size_x(self) -> int:
        """
        Get the width of the occupancy grid.

        Returns:
            int: The width of the occupancy grid.
        """

        return self.occupancy_grid.info.width

    def get_size_y(self) -> int:
        """
        Get the height of the occupancy grid.

        Returns:
            int: The height of the occupancy grid.
        """

        return self.occupancy_grid.info.height

    def map_to_world(self, map_x: int, map_y: int) -> Tuple[float, float]:
        """
        Convert map coordinates to world coordinates.

        Args:
            map_x (int): The x-coordinate on the map.
            map_y (int): The y-coordinate on the map.

        Returns:
            Tuple[float, float]: The world coordinates corresponding to the map coordinates.
        """

        world_x = (
            self.occupancy_grid.info.origin.position.x + (map_x + 0.5) * self.occupancy_grid.info.resolution
        )
        world_y = (
            self.occupancy_grid.info.origin.position.y + (map_y + 0.5) * self.occupancy_grid.info.resolution
        )

        return world_x, world_y

    def world_to_map(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to map coordinates.

        Args:
            world_x (float): The x-coordinate in world coordinates.
            world_y (float): The y-coordinate in world coordinates.

        Returns:
            Tuple[int, int]: The map coordinates corresponding to the world coordinates.
        """

        if world_x < self.occupancy_grid.info.origin.position.x or world_y < self.occupancy_grid.info.origin.position.y:
            raise Exception("World coordinates out of bounds")

        map_x = int((world_x - self.occupancy_grid.info.origin.position.x) / self.occupancy_grid.info.resolution)
        map_y = int((world_y - self.occupancy_grid.info.origin.position.y) / self.occupancy_grid.info.resolution)

        if map_y > self.occupancy_grid.info.height or map_x > self.occupancy_grid.info.width:
            raise Exception("Out of bounds")

        return map_x, map_y

    def __get_index(self, map_x: int, map_y: int) -> int:
        """
        Calculate the index in the occupancy grid data array for the given map coordinates.

        Args:
            map_x (int): The x-coordinate on the map.
            map_y (int): The y-coordinate on the map.

        Returns:
            int: The index in the occupancy grid data array.
        """

        return map_y * self.occupancy_grid.info.width + map_x
