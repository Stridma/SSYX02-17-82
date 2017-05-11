#!/usr/bin/env python

"""
This file contains a lot of functions. The idea is that everything regarding controlling the robots when running a
simulation is found here. To make things clearer it is divided into three parts: the first part contains functions for
moving the robots into formation, the second part contains functions to move a leader to a goal while the other robots,
named "followers", are trying to keep the right distance to the leader but at the same time moving in the direction of
the leader, and lastly the third part is just a common function to move a robot to a position, which is a function
both the first parts have in common.
"""

import rospy
import numpy
from nav_msgs.msg import Odometry
import transformCoordinates
import math
import PID
import angles
import time
import astar
import vectors
import leader_assignment
import robot_to_node_assignment
from geometry_msgs.msg import Twist
twist = Twist()

# ------------------------------------------------------- PART 1 -------------------------------------------------------
# ------------------------------- Calculations of movement to reach the initial formation ------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# --- START OF BAD CODE ---
# This should be done by using a thread instead
global nodes_assigned
global obstacles
obstacles =[[4, 0.5], [0, 5]]
nodes_assigned = False
global at_goal
at_goal = False
global iterations
iterations = 0
#global matrix
#matrix = [[0 for i in range(3)] for j in range(600)]
# --- END OF BAD CODE ---


# Function that subscribes to robots' positions and call a function to take action when updates are received. This
# function runs until the robots are in the desired formation.

def initial_formation_controller(robots, goal_positions):
    global at_goal
    at_goal = False
    # Make sure that the code at least runs one time, and if they are in position they will notify the system
    # themselves.
    robots[0].set_at_position(False)
    robots[1].set_at_position(False)
    robots[2].set_at_position(False)

    # Subscribe to position data from all robots in system
    subscribers = [0] * 3
    for i in range(0, len(robots)):
        subscribers[i] = rospy.Subscriber(robots[i].name + "/pose", Odometry, formation_callback,
                                          (robots[i], robots, goal_positions), queue_size=1)

    # Run until all robots are in position
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            at_goal = True
            # When all robots are at the goal in formation, make sure they stop moving by adding a time delay.
            time.sleep(1)
            break

    print "We're in formation!"

    # Unsubscribe to the nodes in the system when done.
    for i in range(0, 3):
        subscribers[i].unregister()

    # Take a short break
    rospy.sleep(0.5)


# Callback function that listens to position data of one robot. Needs a list of the goal positions for all robots as
# well as what robot that called it. Also needs a list of all robots that are currently in the system.

def formation_callback(data, args):
    global nodes_assigned
    robot = args[0]
    robots = args[1]
    goal_positions = args[2]

    robot.set_position(transformCoordinates.room_coordinates(data))
    robot.set_orientation(transformCoordinates.from_quaternion_to_radians(data))

    # Where does the robot belong? Using a thread might be neater, but this allows only one calculation being done at
    # the same time.
    if not nodes_assigned:
        nodes_assigned = True
        simulation = True
        robot_to_node_assignment.assign_nodes(robots, goal_positions, simulation)
        nodes_assigned = False

    # Find this robot's goal.
    goal_position = goal_positions[robot.get_node_in_system()]

    # Move it to the goal, but not too fast.
    max_speed = 2
    move_robot_to_goal(robot, goal_position, max_speed)


# ------------------------------------------------------- PART 2 -------------------------------------------------------
# ----------------------------------- Calculations of the movement of the formation ------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function to move the formation. Subscribes to the leader separately as it has another callback function, the leader
# does not care at all about the other robots but instead just moves towards its goal. All robots in the system apart
# from the leader are defined as followers.

def move_formation_controller(robots, goal, distances):
    global at_goal
    global iterations
    iterations = 0
    at_goal = False

    # Make sure that the code at least runs one time, and if they are in position they will notify the system
    # themselves.
    robots[0].set_at_position(False)
    robots[1].set_at_position(False)
    robots[2].set_at_position(False)

    # Find the shortest path for the leader to the goal. Set an area in which the robots move below. (Preferably equal
    # to the area you are using for the simulations).
    minx = -10
    miny = -10
    maxx = 10
    maxy = 10
    path = find_path(goal, robots, minx, maxx, miny, maxy)

    # The variables for the PID-controller have to be manually adjusted in order to make the robots moving as efficient
    # as possible. You have to define "efficient" yourself.
    # For the movement of the formation the followers have to consider the distance to both the other robots in the
    # system. Therefore the followers use two PID's each, one for keeping the right distance from the leader and one for
    # the other follower. Maybe it's beneficial to have a higher gain for getting closer to the leader as it is moving
    # on its own, while both of the followers try to get closer to each other. This is totally up to you though.
    # Naming convention: pid_"ROBOT"_"TARGET".
    pid_follower_1_follower = PID.PidController(0.8, 0.0008, 0.9)
    pid_follower_1_leader = PID.PidController(0.8, 0.0008, 0.9)
    pid_follower_2_follower = PID.PidController(0.8, 0.0008, 0.9)
    pid_follower_2_leader = PID.PidController(0.8, 0.0008, 0.9)
    pid_list = [pid_follower_1_leader, pid_follower_1_follower, pid_follower_2_leader, pid_follower_2_follower]

    # Subscribe to the position and orientation data from the robots.
    sub = [0] * 3
    for i in range(0, 3):
        sub[i] = rospy.Subscriber(robots[i].name + "/pose", Odometry, robot_callback,
                                  (robots[i], path, distances, robots, pid_list))

    # Make sure it keeps running until the robots are in position.
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            at_goal = True
            time.sleep(1)
            break

    print "I'm in formation at goal"

    # Unsubscribe from the nodes since we are done.
    for i in range(0, 3):
        sub[i].unregister()

    # Take a short break
    rospy.sleep(0.5)


# Callback function for one robot's position and orientation data. The function decides what to do with the robot using
# information about this robot, the other robots, what distances should be in between and what path the leader should
# take to reach the goal.

def robot_callback(data, args):
    robot = args[0]
    path = args[1]
    distances = args[2]
    robots = args[3]
    pid_list = args[4]

    # Get the position and orientation data in "useful" representation.
    robot.set_position(transformCoordinates.room_coordinates(data))
    robot.set_orientation(transformCoordinates.from_quaternion_to_radians(data))

    # Leader is 0 in system, followers are 1 and 2.
    if robot.get_node_in_system() == 0:
        # It might be desired to have a maximum speed of the leader, so that the followers
        # are given a reasonable chance to keep the formation.
        max_speed = 0.15
        move_leader(robot, path, max_speed)
    elif robot.get_node_in_system() == 1:
        move_follower(robot, distances, robots, pid_list[0], pid_list[1])
    elif robot.get_node_in_system() == 2:
        move_follower(robot, distances, robots, pid_list[2], pid_list[3])
    else:
        print "---WARNING--- THIS ROBOTS IS NEITHER LEADER NOR FOLLOWER"


# ------------------------------------------------------- PART 3 -------------------------------------------------------
# --------------------------------------------------- Moving a robot ---------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# A simple movement algorithm that moves the robot smoothly to a goal. Need information about the robots state, e.g. its
# position and orientation which it gets from "data". The information has to be converted into "normal" positioning data
# , in this case a position in [x,y] coordinates and an orientation in radians, calculated in the ACW direction from the
# positive x-axis.

def move_robot_to_goal(robot, goal_position, max_speed):
    global at_goal
    # Position of robot in format [x,y]
    robot_position = robot.get_position_simulation()

    # Orientation of robot in radians from 0 to 2pi
    robot_orientation = robot.get_orientation()

    # The angle of the goal position given in radians in the same way as the orientation
    # of the robot.
    goal_angle = angles.angle_between_points(goal_position, robot_position)
    target_angle = angles.angle_difference(goal_angle, robot_orientation)
    distance = vectors.distance_points(goal_position, robot_position)

    # A tolerance level to decide whether the robot is at the goal or not.
    tol = 0.1

    twist.angular.z = 0.5 * target_angle
    twist.linear.x = distance* ((math.pi - math.fabs(target_angle)) / math.pi) ** 2

    if distance < tol:
        robot.set_at_position(True)
        twist.angular.z = 0
    elif distance >= tol:
        robot.set_at_position(False)

    # Make sure the robot is not moving too fast and neither backwards.
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed
    elif twist.linear.x < 0:
        twist.linear.x = 0

    # If all robots are at goal, then stop.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    robot.pub.publish(twist)


# Like above, but moves a robot to a list of goal points.

def move_leader(robot, goal, max_speed):
    global iterations
    goal_position = goal[iterations]
    print goal_position
    global at_goal
    # Position of robot in format [x,y]
    robot_position = robot.get_position_simulation()

    # Orientation of robot in radians from 0 to 2pi
    robot_orientation = robot.get_orientation()

    # The angle of the goal position given in radians in the same way as the orientation
    # of the robot.
    goal_ang = angles.angle_between_points(goal_position, robot_position)
    tar_ang = angles.angle_difference(goal_ang, robot_orientation)

    # Distance to the goal from current position.
    distance = vectors.distance_points(goal_position, robot_position)

    # A tolerance level to decide whether the robot is at the goal or not.
    tol = 0.1

    twist.angular.z = 0.5 * tar_ang
    if iterations == len(goal) - 1:
        twist.linear.x = distance * ((math.pi - math.fabs(tar_ang)) / math.pi) ** 2
    else:
        twist.linear.x = max_speed * ((math.pi - math.fabs(tar_ang)) / math.pi) ** 2

    if distance < tol:
        if iterations == len(goal) - 1:
            robot.set_at_position(True)
            twist.angular.z = 0
        else:
            iterations += 1
    elif distance >= tol:
        robot.set_at_position(False)

    # Make sure the robot is not moving too fast and neither backwards.
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed
    elif twist.linear.x < 0:
        twist.linear.x = 0

    # If all robots are at goal, then stop.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    robot.pub.publish(twist)

    # This code can be uncommented/commented if you would like to store information about the robots so that you can use
    # it later.
    """
    file = open("state_leader.txt", "a")
    file.write(str(robot_position[0]) + " ")
    file.write(str(robot_position[1]) + " ")
    file.write(str(twist.linear.x) + " ")
    file.write(str(twist.angular.z) + " ")
    file.write(str(time.time()) + " ")
    file.write("\n")
    file.close()
    """


# Movement function for the followers. When receiving an update of a followers position
# it calculates in what direction the follower should go.

def move_follower(robot, distances, robots, pid_follower, pid_leader):
    global at_goal

    # Find what the other robots are in the system.
    for i in range(0, len(robots)):
        if robots[i].get_node_in_system() == 0:
            leader = robots[i]
        elif (robots[i].get_node_in_system() == 1 or robots[i].get_node_in_system() == 2) and (robots[i] is not robot):
            follower = robots[i]

    # Desired distances to keep to the other robots. Leader and follower are either correctly assigned or something else
    # is terribly wrong. Shouldn't initialise them as a safety since that wouldn't allow you to identify the problem.
    desired_distance_leader = distances[leader.get_node_in_system()][robot.get_node_in_system()]
    desired_distance_follower = distances[follower.get_node_in_system()][robot.get_node_in_system()]

    # Position of robot in format [x,y]
    robot_position = robot.get_position_simulation()

    # State of the leader and other follower
    leader_position = leader.get_position_simulation()
    leader_orientation = leader.get_orientation()

    follower_position = follower.get_position_simulation()

    # Calculate the actual distances to the other robots.
    distance_leader = vectors.distance_points(leader_position, robot_position)
    distance_follower = vectors.distance_points(follower_position, robot_position)

    # Orientation of robot in radians from 0 to 2pi
    robot_orientation = robot.get_orientation()

    # Use PID-controller to go to a position which is at the desired distance from both the other robots.
    error_leader = distance_leader - desired_distance_leader
    error_follower = distance_follower - desired_distance_follower
    u_leader = pid_leader.pid(error_leader)
    u_follower = pid_follower.pid(error_follower)

    # u is always going to be a positive value. As long as the robots don't overshoot this shouldn't be a problem, but
    # if they do they might keep moving and even crash into the leader. Have this in consideration.
    u = math.sqrt(u_follower ** 2 + u_leader ** 2)

    # Create vectors to the leader, the other follower, the orientation of the leader and from these decide on a
    # direction vector, which this follower should move along. The further away the robot is from either the leader or
    # the other follower, the bigger the tendency is to move towards that robot. If it's more or less at the right
    # distance from the other robots it is favorable to move in the orientation of the leader, which is implemented by
    # the usage of the variable scale which is an exponential decaying function squared and scaled by 2/3, in other
    # words the follower will only move in the direction of the leader's orientation if it's more or less perfectly in
    # the right position.
    if u_leader != 0:
        vector_leader = vectors.multiply(vectors.normalise(vectors.subtract(leader_position, robot_position)),
                                         u_leader * u_follower)
    else:
        vector_leader = vectors.multiply(vectors.normalise(vectors.subtract(leader_position, robot_position)), 0.001)
    if u_follower != 0:
        vector_follower = vectors.multiply(vectors.normalise(vectors.subtract(follower_position, robot_position)),
                                           u_follower)
    else:
        vector_follower = vectors.multiply(vectors.normalise(vectors.subtract(follower_position, robot_position)),
                                           0.001)

    scale = math.exp(-u) ** 2 * 2 / 3
    vector_orientation_leader = vectors.multiply([math.cos(leader_orientation), math.sin(leader_orientation)], scale)

    direction_vector = vectors.add(vectors.add(vector_follower, vector_leader), vector_orientation_leader)

    # Calculate a goal angle from the direction vector.
    goal_angle = math.atan2(direction_vector[1], direction_vector[0])
    if goal_angle < 0:
        goal_angle += 2 * math.pi

    # Calculate a target angle from the goal angle and the orientation of this robot.
    target_angle = angles.angle_difference(goal_angle, robot_orientation)

    # Spin the robot towards the desired orientation. In general a P-regulator should be enough.
    twist.angular.z = target_angle * 0.5

    # Move the robot forward. The further away it is from the goal, as well as earlier error and predicted future error
    # by the PID is considered in the variable u. Also the robot will move by full speed when oriented correctly, but
    # slower the further away it is from its desired orientation given by target_angle.
    twist.linear.x = u * math.fabs(math.pi - math.fabs(target_angle)) / math.pi

    # If the robot is within an acceptable range, named tol, it is considered "in position". The robot will still keep
    # moving though until all robots are at the goal, which is taken care of later.
    tol = 0.1
    if math.fabs(error_follower) < tol and math.fabs(error_leader) < tol:
        robot.set_at_position(True)
    else:
        robot.set_at_position(False)

    # Make sure the robot is not moving too fast. What's too fast is up to you.
    max_speed = 0.3
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed

    # If we get a negative control commando from the leader PID, it means we are getting too close. since we don't want
    # to stop fully it might be nice to have the follower going really slowly.
    if u_leader < 0:
        twist.linear.x = 0.07

    # If there is a problem with the follower crashing into the leader this could avoid it, but it would also mean there
    # is a risk of stopping - moving - stopping etc. If the PID is used correctly overshooting shouldn't be a problem,
    # which means it never gets too close. Uncomment if necessary.
    # if distance_leader < 1:
    #    twist.angular.z = 0
    #    twist.linear.x = 0

    # If all the robots are at goal we have to stop moving of course.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    robot.pub.publish(twist)

    # This code can be uncommented/commented if you would like to store information about the robots so that you can use
    # it later.
    """
    if robot.get_node_in_system() == 1:
        file = open("state_follower1.txt", "a")
        file.write(str(robot_position[0]) + " ")
        file.write(str(robot_position[1]) + " ")
        file.write(str(twist.linear.x) + " ")
        file.write(str(twist.angular.z) + " ")
        file.write(str(time.time()) + " ")
        file.write("\n")
        file.close()
    else:
        file = open("state_follower2.txt", "a")
        file.write(str(robot_position[0]) + " ")
        file.write(str(robot_position[1]) + " ")
        file.write(str(twist.linear.x) + " ")
        file.write(str(twist.angular.z) + " ")
        file.write(str(time.time()) + " ")
        file.write("\n")
        file.close()
    """


# ------------------------------------------------------- Part 4 -------------------------------------------------------
# -------------------------------------------- Move robots into orientation --------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function to rotate the robots into a goal orientation defined by the one calling the function.

def robot_orientation(robots, goal_orientation):
    for i in range(0, len(robots)):
        robots[i].set_at_position(False)

    # Subscribe to position data from all robots in system
    subscribers = [0] * 3
    for i in range(0, len(robots)):
        subscribers[i] = rospy.Subscriber(robots[i].name + "/pose", Odometry, orientation_callback,
                                          (robots[i], goal_orientation), queue_size=1)

    # Make sure it keeps running. Should terminate when robots are in the correct orientation.
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            break

    print "We're in the correct orientation! "

    for i in range(0, 3):
        subscribers[i].unregister()

    # Take a short break
    rospy.sleep(0.5)


# Callback function which uses the internal coordinate system to rotate the robots into the correct orientation.

def orientation_callback(data, args):
    robot = args[0]
    goal_orientation = args[1]
    tol = 0.01  # Tolerance in radians for accepting the robot as being in the correct orientation.

    orientation = transformCoordinates.from_quaternion_to_radians(data)

    if orientation < 0:
        orientation += 2 * math.pi

    target_angle = angles.angle_difference(goal_orientation, orientation)

    if math.fabs(target_angle) > tol:
        twist.angular.z = target_angle
        robot.set_at_position(False)
    else:
        twist.angular.z = 0
        robot.set_at_position(True)

    robot.pub.publish(twist)


# ------------------------------------------------------- Part 5 -------------------------------------------------------
# ---------------------------------------------------- Path finding ----------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function that creates a matrix representing the simulation environment area and then calls the A* algorithm to find
# the shortest path to the goal from the leader in this matrix.

def find_path(goal, robots, minx, maxx, miny, maxy):

    # Set the robot closest to the target as leader. Either check them here or go to a formation at first.
    leader_assignment.set_leader(goal, robots, True)

    # Find the robots' positions.
    for i in range(0, len(robots)):
        if robots[i].get_node_in_system() == 0:
            leader_position = robots[i].get_position_simulation()
        elif robots[i].get_node_in_system() == 1:
            follower_1_position = robots[i].get_position_simulation()
        elif robots[i].get_node_in_system() == 2:
            follower_2_position = robots[i].get_position_simulation()

    # Now let's create a matrix representing the space in which the robots operate.

    # 10 intervals per meter
    scale = 10
    nx = int((maxx - minx) * scale)
    ny = int((maxy - miny) * scale)

    matrix = [[0 for i in range(ny)] for j in range(nx)]

    # Place the goal and start in the matrix (or get their representation at least).
    start_matrix = (int((leader_position[0] - minx) * scale), int((leader_position[1] - miny) * scale))
    goal_matrix = (int((goal[0] - minx) * scale), int((goal[1] - miny) * scale))

    # Place followers in matrix and add some margins around them so that the leader at least not is aiming straight
    # towards them.
    follower_1_position_matrix = [int((follower_1_position[0] - minx) * scale),
                                  int((follower_1_position[1] - miny) * scale)]
    follower_2_position_matrix = [int((follower_2_position[0] - minx) * scale),
                                  int((follower_2_position[1] - miny) * scale)]

    # How big are they (in meters * scale)?
    width_follower = 0.8 * scale
    for i in range(nx):
        for j in range(ny):
            if vectors.distance_points(follower_1_position_matrix, [i,j]) <= width_follower or \
                            vectors.distance_points(follower_2_position_matrix, [i, j]) <= width_follower:
                matrix[i][j] = 1

    # Place obstacles in matrix and add some space around them so that they are not just one matrix element wide.
    global obstacles
    width_obstacle = 1.5 * scale
    if obstacles[0] != [0]:
        obstacle_0_position_matrix = [int((obstacles[0][0] - minx) * scale),
                                      int((obstacles[0][1] - miny) * scale)]
        print obstacle_0_position_matrix
        for i in range(nx):
            for j in range(ny):
                if vectors.distance_points(obstacle_0_position_matrix, [i,j]) <= width_obstacle:
                    matrix[i][j] = 1

    if obstacles[1] != [0]:
        obstacle_1_position_matrix = [int((obstacles[1][0] - minx) * scale),
                                      int((obstacles[1][1] - miny) * scale)]
        for i in range(nx):
            for j in range(ny):
                if vectors.distance_points(obstacle_1_position_matrix, [i,j]) <= width_obstacle:
                    matrix[i][j] = 1

    # Find the shortest path using the A* algorithm
    sim_area = numpy.array(matrix)
    path = astar.astar(sim_area, goal_matrix, start_matrix)

    # Shift the path so that it corresponds to the actual area that is considered.
    if path is not False:
        for i in range(len(path)):
            path[i] = vectors.add(vectors.multiply(path[i], 1. / scale), [minx, miny])

    # It seems to be a good idea to not tell the robot to go to a point really close to it, so just cut out the first
    # 5 or so points. Also add the goal position at the end since it is not given from the A* algorithm.
    short_path = [0 for i in range(len(path)-4)]
    for i in range(4, len(path)-1):
        short_path[i-4]=path[i]
    short_path[len(short_path) - 1] = goal

    return short_path
