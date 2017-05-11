#!/usr/bin/env python

"""
A class that takes care of a run of the real robots. If an object is created it contains information like what robots
are in the system.
"""

import rospy
import controller
import robot
import formations


# The run class from which it is possible to create simulation objects.

class Run:

    # Initialise simulation, set up number of robots and set formation
    def __init__(self, nbr_of_nodes):

        # User need to specify number of robots
        self.nbr_of_nodes = nbr_of_nodes

        # Initiate a list of nodes running
        self.nodes = [0] * self.nbr_of_nodes

        # Initiate the robots and save their representation to the list
        for i in range(0, self.nbr_of_nodes):
            self.nodes[i] = robot.Robot("Robot" + i.__str__(), i)

        # Set formation to a line as default
        self.formation = formations.set_formation("line")

    # Function to set formation of robots. Needs user interaction.
    def set_formation(self):
        self.formation = formations.set_formation(raw_input("Input formation: "))

    # Function to move robots to the formation of the system.
    def go_to_formation(self):
        controller.initial_formation_controller(self.nodes, self.formation)

    # Function to move the entire formation, needs a goal for the leader and what distances it should keep to the other
    # robots. The distance matrix should contain information such that distances[a][b] gives the distance from robot
    # a to robot b.
    def move_formation(self, goal, distances):
        rospy.sleep(1)
        controller.move_formation_controller(self.nodes, goal, distances)

    # Function to rotate the robots in the system to a desired orientation called goal_angle, which is given from the
    # positive x-axis in ACW direction.
    def rotate(self, goal_angle):
        controller.robot_orientation(self.nodes, goal_angle)
