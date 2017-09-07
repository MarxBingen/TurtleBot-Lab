#!/usr/bin/env python
"""Module for Heading Stuff"""
from enum import Enum
import math
import tf

class SimpleHeading(Enum):
    """A Simple Class for dealing with simple Headings"""
    NORD = 1
    OST = 2
    SUED = 3
    WEST = 4

    @staticmethod
    def from_degrees360(degrees):
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
    def from_quaternion(quat):
        (r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        y =  math.degrees(y)+180
        if 225 >= y > 135:
            return SimpleHeading.NORD
        if 135 >= y > 45:
            return SimpleHeading.OST
        if 315 >= y > 225:
            return SimpleHeading.WEST
        if 315 < y or y < 45:
            return SimpleHeading.SUED
        return SimpleHeading.NORD

    @staticmethod
    def yaw(heading):
        """Returns yaw from current heading in degrees 0-359"""
        new_yaw = 0
        if heading is SimpleHeading.NORD:
            new_yaw = 180
        elif heading is SimpleHeading.WEST:
            new_yaw = 270
        elif heading is SimpleHeading.SUED:
            new_yaw = 0
        elif heading is SimpleHeading.OST:
            new_yaw = 90
        return new_yaw
