#!/usr/bin/env python

"""
This file contains function that calculates things related to angles in 2D.
As of March 17 2017 it includes:
*Calculating the angle between two points.
*Calculating the difference between two angles.
"""

import math


# This function returns the angle between two points in 2D.
# The angle is returned in radians and is given from 0 to 2pi

def angle_between_points(point_1, point_2):
    angle = math.atan2(point_1[1] - point_2[1], point_1[0] - point_2[0])
    if angle < 0:
        angle += 2 * math.pi
    return angle


# This function calculates the difference between two angles in 2D.
# The difference is given in radians spanning from -pi to pi

def angle_difference(angle_1, angle_2):
    angle = (angle_1 - angle_2)
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle
