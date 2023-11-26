#!/usr/bin/env python3

class FrontierPoint:
    """
    Class representing a point on the frontier of a map.

    Attributes:
        classification (int): The classification of the frontier point.
        map_x (int): The x-coordinate of the frontier point on the map.
        map_y (int): The y-coordinate of the frontier point on the map.
    """

    def __init__(self, x: int, y: int):
        """
        Initialize a FrontierPoint with specified map coordinates.

        Args:
            x (int): The x-coordinate of the frontier point.
            y (int): The y-coordinate of the frontier point.
        """
        self.classification = 0
        self.map_x = x
        self.map_y = y
