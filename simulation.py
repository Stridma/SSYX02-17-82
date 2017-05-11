#!/usr/bin/env python

"""
This file contains a simulation class, from which it is possible to create instances of simulations. Most likely only
one simulation will be ran at each time, but it is convenient to store variables belonging to a simulation in an object.

The simulation has functions to set a formation, to go to that formation, to move the formation and of course a
constructor to initialise objects.
"""

import rospy
import simulation_controller
import robot
import formations


# The simulation class from which it is possible to create simulation objects.

class Simulation:

    # Initialise simulation, set up number of robots and set formation
    def __init__(self, nbr_of_nodes):

        # User need to specify number of robots
        self.nbr_of_nodes = nbr_of_nodes

        # Initiate a list of nodes running
        self.nodes = [0] * self.nbr_of_nodes

        # Initiate the robots and save their representation to the list
        for i in range(0, self.nbr_of_nodes):
            self.nodes[i] = robot.Robot("/Robot" + i.__str__(), i)

        # Set formation to a line as default
        self.formation = formations.set_formation("line")

    # Function to set formation of robots. Needs user interaction.
    def set_formation(self):
        self.formation = formations.set_formation(raw_input("Input formation: "))

    # Function to move robots to the formation of the system.
    def go_to_formation(self):
        simulation_controller.initial_formation_controller(self.nodes, self.formation)

    # Function to move the entire formation, needs a goal for the leader and what distances it should keep to the other
    # robots. The distance matrix should contain information such that distances[a][b] gives the distance from robot
    # a to robot b.
    def move_formation(self, goal, distances):
        rospy.sleep(1)
        simulation_controller.move_formation_controller(self.nodes, goal, distances)

    # Function to rotate the robots in the system to a desired orientation called goal_angle, which is given from the
    # positive x-axis in ACW direction.
    def rotate(self, goal_angle):
        simulation_controller.robot_orientation(self.nodes, goal_angle)
