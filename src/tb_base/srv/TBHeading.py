#!/usr/bin/env python
"""Module for Heading Stuff"""
from enum import Enum
import math

class SimpleHeading(Enum):
    """A Simple Class for dealing with simple Headings"""
    NORD = 1
    OST = 2
    SUED = 3
    WEST = 4

    @staticmethod
    def from_degrees(degrees):
        """Changes heading depending on richtung"""
        newheading = SimpleHeading.NORD
        if degrees >= 315 and degrees < 45:
            return SimpleHeading.NORD
        if degrees >= 45 and degrees < 135:
            return SimpleHeading.OST
        if degrees >= 135 and degrees < 225:
            return SimpleHeading.SUED
        if degrees >= 225 and degrees < 315:
            return SimpleHeading.WEST
        return newheading

    @staticmethod
    def yaw(heading):
        """Returns yaw from current heading"""
        new_yaw = 0
        if heading is SimpleHeading.NORD:
            new_yaw = math.pi / 2.0
        elif heading is SimpleHeading.WEST:
            new_yaw = math.pi
        elif heading is SimpleHeading.SUED:
            new_yaw = -math.pi / 2.0
        elif heading is SimpleHeading.OST:
            new_yaw = 2 * math.pi
        return new_yaw
