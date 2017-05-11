"""
This is a function containing the translation of robot-to-node assignment, which is initially modelled as an LP-problem.
It requires no installation of additional modules or packages. The model used is based around a matrix containing all
the possible distances from every robot to every node in the desired formation, where the robots are on the rows and the
nodes are on the columns of the matrix, such that d_ij is the distance from the i-th robot's current position to the
j-th node in the formation. Please see final report for additional mathematical foundation for the problem.

***IMPORTANT NOTE: This algorithm is customized to fit 3 robots and 3 nodes. The mathematical model is applicable to n
nodes and n robots, but the execution would require the installation of a more powerful library or package, such as PuLP
or scipy.
It is also important to note that 0-indexation is applied (just in case you didn't take that for granted), thus the
first robot would be robot_0 and so on. The least complex solution is to input the distances individually from where the
algorithm is called on, but it could easily be made to accept a list and sort the distances internally.
***OBS: The function returns a dictionary, where the robot-to-node assignment is
done for each robot and node as "robot": "node".
Let me know if further documentation is desired. // Neda

UPDATE: MARCH 17 2017
The code is mostly the same, but it does the assignment just using knowledge about the robots in the system and the
nodes which represents the goals. Also it sets the states of the robots without returning anything to the caller.
"""

import vectors


# Input robots and nodes in system and whether it is a simulation or not.

def assign_nodes(robots, nodes, simulation):

    # These are the cost variables, one for each robot
    c_0 = 1
    c_1 = 1
    c_2 = 1

    # Distances is a matrix containing information about the distance between
    # all robots to all nodes, represented as:
    # Distance from robot A to node B = distances[A][B]
    distances = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    if simulation:
        for i in range(0, len(robots)):
            for j in range(0, len(nodes)):
                distances[i][j] = vectors.distance_points(robots[i].get_position_simulation(), nodes[j])
    else:
        for i in range(0, len(robots)):
            for j in range(0, len(nodes)):
                distances[i][j] = vectors.distance_points(robots[i].get_position(), nodes[j])

    # These are all potential outcomes of the robot-to-node assignment, calculated as an overall cost with distance
    # travelled * cost variable. It is designed so that no two robots go to the same node, and no robot goes to more
    # than one node (duh).
    # *** OBS *** CALCULATED MANUALLY AND SHOULD NOT BE TAMPERED WITH
    # ------------------------------------------------------------------------------------------------------------------
    #                  *** YOU ARE NOW ENTERING THE DANGER ZONE ***
    # ------------------------------------------------------------------------------------------------------------------
    v1 = distances[0][0] * c_0 + distances[1][1] * c_1 + distances[2][2] * c_2
    v2 = distances[2][0] * c_2 + distances[1][1] * c_1 + distances[0][2] * c_0
    v3 = distances[1][0] * c_1 + distances[2][1] * c_2 + distances[0][2] * c_0
    v4 = distances[0][0] * c_0 + distances[2][1] * c_2 + distances[1][2] * c_1
    v5 = distances[1][0] * c_1 + distances[0][1] * c_0 + distances[2][2] * c_2
    v6 = distances[2][0] * c_2 + distances[0][1] * c_0 + distances[1][2] * c_1
    # ------------------------------------------------------------------------------------------------------------------
    #                  *** YOU ARE NOW LEAVING THE DANGER ZONE ***
    # ------------------------------------------------------------------------------------------------------------------

    # List used in determination of which "version" offers minimal cost
    list_of_distances = [v1, v2, v3, v4, v5, v6]

    # Variable "version_index" is used in saving minimal cost path and finally returning the correct dictionary. Should
    # not be modified!
    version_index = 1

    # Minimization happens here. It should not be modified.
    for i in range(0, 6):
        if list_of_distances[i] < list_of_distances[version_index-1]:
            version_index = i+1

    # Dictionary containing all possible final robot-to-node assignments. The dictionary contains 6 dictionaries, where
    # each internal dictionary is a possible outcome of the assignment.
    # *** OBS *** CALCULATED MANUALLY AND SHOULD NOT BE TAMPERED WITH
    # -------------------------------------------------------------------
    #          *** YOU ARE NOW ENTERING THE DANGER ZONE ***
    # --------------------------------------------------------------------
    dict = {1: {"robot_0": 0, "robot_1": 1, "robot_2": 2},
            2: {"robot_0": 2, "robot_1": 1, "robot_2": 0},
            3: {"robot_0": 2, "robot_1": 0, "robot_2": 1},
            4: {"robot_0": 0, "robot_1": 2, "robot_2": 1},
            5: {"robot_0": 1, "robot_1": 0, "robot_2": 2},
            6: {"robot_0": 1, "robot_1": 2, "robot_2": 0}}
    # -------------------------------------------------------------------
    #          *** YOU ARE NOW LEAVING THE DANGER ZONE ***
    # --------------------------------------------------------------------

    nodes = dict.get(version_index)
    robots[0].set_node_in_system(nodes.get("robot_0"))
    robots[1].set_node_in_system(nodes.get("robot_1"))
    robots[2].set_node_in_system(nodes.get("robot_2"))
