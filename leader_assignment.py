import vectors


# Sets the robot closest to the goal as leader and the others as followers. Being follower 1 or 2 does not matter.

def set_leader(goal, robots, simulation):
    positions = [0] * len(robots)
    for i in range(0, len(robots)):
        if simulation:
            positions[i] = robots[i].get_position_simulation()
        else:
            positions[i] = robots[i].get_position()

    if vectors.distance_points(goal, positions[0]) < vectors.distance_points(goal, positions[1]) and \
                 vectors.distance_points(goal, positions[0]) < vectors.distance_points(goal, positions[2]):
        robots[0].set_node_in_system(0)
        robots[1].set_node_in_system(1)
        robots[2].set_node_in_system(2)

    elif vectors.distance_points(goal, positions[1]) < vectors.distance_points(goal, positions[0]) and \
                 vectors.distance_points(goal, positions[1]) < vectors.distance_points(goal, positions[2]):
        robots[0].set_node_in_system(1)
        robots[1].set_node_in_system(0)
        robots[2].set_node_in_system(2)

    else:
        robots[0].set_node_in_system(2)
        robots[1].set_node_in_system(1)
        robots[2].set_node_in_system(0)
