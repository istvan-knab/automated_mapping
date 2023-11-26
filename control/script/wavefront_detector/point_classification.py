#!/usr/bin/env python3

from enum import Enum


class PointClassification(Enum):
    """
        This enumeration is used to classify points on a map into different categories.

        Attributes:
            MapOpen: Represents an open point on the map.
            MapClosed: Represents a closed (obstacle) point on the map.
            FrontierOpen: Represents an open point on the frontier.
            FrontierClosed: Represents a closed (visited) point on the frontier.
    """

    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8
