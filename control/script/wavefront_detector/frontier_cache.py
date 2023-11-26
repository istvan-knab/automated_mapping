#!/usr/bin/env python3

from frontier_point import FrontierPoint


class FrontierCache:
    """
    This class provides a cache to store and retrieve frontier points based on their coordinates.

    Attributes:
        cache (dict): A dictionary to store frontier points with their corresponding coordinates.
    """

    cache = {}

    def get_frontier_point(self, x: int, y: int) -> 'FrontierPoint':
        """
        Retrieve a frontier point from the cache based on coordinates.

        Args:
            x (int): The x-coordinate of the frontier point.
            y (int): The y-coordinate of the frontier point.

        Returns:
            FrontierPoint: The frontier point object associated with the coordinates (x, y).
        """
        idx = self.__cantor_hash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        # If the frontier point is not in the cache, create a new one and add it to the cache
        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    @staticmethod
    def __cantor_hash(x: int, y: int) -> float:
        """
        Cantor pairing function to create a unique hash value for given coordinates.

        Args:
            x (int): The x-coordinate.
            y (int): The y-coordinate.

        Returns:
            float: The unique hash value for the coordinates (x, y).
        """
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear_cache(self) -> None:
        """
        Clear the cache, removing all stored frontier points.

        Returns:
            None
        """
        self.cache = {}
