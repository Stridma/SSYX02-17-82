#!/usr/bin/env python

"""
This file contains the Robot class which is used to store information about robots. The idea is that the object has
knowledge about its state. Also, it knows how to change the state by using its own "publisher". The robot has no idea
about calculating speeds, direction etc., all this is done in the controller.
"""


import rospy
from geometry_msgs.msg import Twist


# Robot class which can be used to create robot objects

class Robot:

    # Constructor to initiate robot. It needs a name and
    # what robot in the system it corresponds to.

    def __init__(self, name, i):
        self.name = name
        self.node_in_system = i
        self.at_position = False
        self.position = [0, 0]
        self.front_position = [0.1, 0]
        self.orientation = 0

        # Create velocity publisher
        self.pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        # Make new node, make sure it's unique (anonymous = True ensures this)
        rospy.init_node("robot", anonymous=True)

    # Function to tell the robot if it is in position or not.
    def set_at_position(self, case):
        self.at_position = case

    # Function that returns whether the robot is in position or not.
    def get_at_position(self):
        return self.at_position

    # Function to change where in the system this robot belongs. Being robot 0 means it is the leader, while other
    # numbers indicate that it is a follower.
    # *** OBS *** USE WITH CAUTION, IF TWO ROBOTS BELONG TO THE SAME NODE THEY MIGHT COLLIDE. IF NEEDED, CHANGE THE
    # NODE OF BOTH ROBOTS.
    def set_node_in_system(self, node):
        self.node_in_system = node

    # Returns what node this robot belongs to in the system.
    def get_node_in_system(self):
        return self.node_in_system

    # Set the position using a 2D coordinate.
    def set_position(self, position):
        self.position = position

    # Set the position of the front tag using a 2D coordinate.
    def set_front_position(self, position):
        self.front_position = position

    # Function that returns the position of the robot in simulations, since the robots are using an internal state
    # representation for simulations and their positions are shifted in relation to each other it is necessary to shift
    # it back when doing calculations. As everything else; this is only to be used with three robots.
    def get_position_simulation(self):
        if self.name == "/Robot0":
            return [self.position[0], self.position[1] + 1]
        elif self.name == "/Robot2":
            return [self.position[0], self.position[1] - 1]
        else:
            return self.position

    # Function that returns the position
    def get_position(self):
        return self.position

    # Function that returns the position of the front tag
    def get_front_position(self):
        return self.front_position

    # Function that sets the orientation of the robot in radians from 0 to 2pi. Calculated from positive x-axis in
    # ACW-direction.
    def set_orientation(self, orientation):
        self.orientation = orientation

    # Returns the orientation of the robot in radians from 0 to 2pi. Calculated from positive x-axis in ACW-direction.
    def get_orientation(self):
        return self.orientation
