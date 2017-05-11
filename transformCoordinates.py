#!/usr/bin/env python

"""
This file contains two functions regarding coordinate transformation. The input is supposed to be of the type "pose",
which is given from the robots internal state. The first function that is found here converts quaternion coordinates
into an angle in radians and the second gives the 2D room coordinates.
"""

import tf
import math


# Function to transform quaternion coordinates into an angle in radians which is
# calculated from the positive x-axis in ACW direction.

def from_quaternion_to_radians(data):
    # Yaw is the only angle of interest in 2D.
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,
                                                                   data.pose.pose.orientation.y,
                                                                   data.pose.pose.orientation.z,
                                                                   data.pose.pose.orientation.w])
    if yaw < 0:
        yaw += 2 * math.pi
    return yaw


# Function that returns the position in format [x,y]

def room_coordinates(data):
    return [data.pose.pose.position.x, data.pose.pose.position.y]
