#!/usr/bin/env python

"""
This file contains the main program that the user is supposed to run. The program can handle both simulations and real
life robots. This file only contains one function, and the purpose is to delegate work to other functions in regard to
the users intentions.

Maybe it would be a nice touch if it had a GUI, but it is of no importance for its functioning.
"""

import rospy
import simulation
import run


# Start program, ask whether it is a simulation
# or not and take action accordingly.

def main():
    sim = raw_input("Simulation (y/n)?: ")
    sim.lower()
    nbr_of_nodes = input("Input number of robots in system: ")
    if sim == "y":
        sim_object = simulation.Simulation(nbr_of_nodes)
    else:
        run_object = run.Run(nbr_of_nodes)

    while True:
        user = raw_input("What do you want to do (h for help)? ")
        user.lower()

        if user == "h" or user == "help":
            print "Possible commands are: "
            print "Go to formation (gtf)"
            print "Move formation (mf)"
            print "Rotate robots (rr)"
            print "Exit"

        elif user == "go to formation" or user == "gtf":
            if sim == "y":
                sim_object.set_formation()
                sim_object.go_to_formation()
            else:
                run_object.set_formation()
                run_object.go_to_formation()

        elif user == "move formation" or user == "mf":
            print "Where do you want the robot to go? "
            goal = [0, 0]
            goal[0] = input("x: ")
            goal[1] = input("y: ")

            print "What should the distances in between them be?"
            distance = input("Distance: ")

            # Distances are given from robot a to b as distances[a][b]
            distances = [[0, distance, distance],
                         [distance, 0, distance],
                         [distance, distance, 0]]
            if sim == "y":
                sim_object.move_formation(goal, distances)
            else:
                run_object.move_formation(goal, distances)

        elif user == "rotate robots" or user == "rr":
            orientation = input("Input orientation in radians: ")
            if sim == "y":
                sim_object.rotate(orientation)
            else:
                run_object.rotate(orientation)

        elif user == "exit" or user == "quit":
            break


# Needed for the program to know that it is supposed to run when called on.
# Catches exception from rospy but takes no action if caught.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass