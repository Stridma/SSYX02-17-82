#!/usr/bin/env python

"""
This file contains a lot of functions. The idea is that everything regarding controlling the robots when running a
simulation is found here. To make things clearer it is divided into four parts: the first part contains functions for
moving the robots into formation, the second part contains functions to move a leader to a goal while the other robots,
named "followers", are trying to keep the right distance to the leader but at the same time moving in the direction of
the leader. The third part is just movement functions for the robots, one for moving a robot to a specific destination
and one for moving followers while keeping the correct distance to other robots. In the fourth part functions are found
that rotates the robots into a desired orientation.

*** IMPORTANT ***
All positions are given in METERS.
"""

# Files written by others
import rospy
import math
import time
import astar
from nav_msgs.msg import Odometry
from gulliview_server.msg import Pos
from geometry_msgs.msg import Twist

# Written modules in this package
import transformCoordinates
import PID
import angles
import vectors
import robot_to_node_assignment

# Renaming of methods to make life easier.
twist = Twist()

# ---start--- ---------------------------------------Global variables--------------------------------------- ---start---
global obstacles
global at_goal
global subscriber
global iterations
iterations = 0
subscriber = None
at_goal = False
obstacles = [0, 0]
# ---end--- -----------------------------------------Global variables----------------------------------------- ---end---


# ------------------------------------------------------- PART 1 -------------------------------------------------------
# ------------------------------- Calculations of movement to reach the initial formation ------------------------------
# ----------------------------------------------------------------------------------------------------------------------


# Function that subscribes to robots' positions and calls a function to take action when updates are received. This
# function runs until the robots are in the desired formation.

def initial_formation_controller(robots, goal_positions):
    global at_goal
    at_goal = False
    # Make sure that the code at least runs one time, and if they are in position they will notify the system
    # themselves.
    robots[0].set_at_position(False)
    robots[1].set_at_position(False)
    robots[2].set_at_position(False)

    # Subscribe to position data from the camera
    subscriber = rospy.Subscriber("/position", Pos, formation_callback, (robots, goal_positions), queue_size=1)

    # Run until all robots are in position
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            at_goal = True
            # When all robots are at the goal in formation, make sure they stop moving by adding a time delay. Not
            # really a problem since the robots stop moving after a while anyways, but always better to be safe
            # than sorry.
            time.sleep(1)
            break

    print "We're in formation! "

    # Unsubscribe to the position data when done.
    subscriber.unregister()

    # Take a short break
    rospy.sleep(0.5)


# Callback function that listens to position data from the camera system. Needs a list of the goal positions for all
# robots as well as what robots that are currently in the system.

def formation_callback(data, args):
    global nodes_assigned
    robots = args[0]
    goal_positions = args[1]

    camera = {data.tagid1: [data.x1, data.y1], data.tagid2: [data.x2, data.y2],
              data.tagid3: [data.x3, data.y3], data.tagid4: [data.x4, data.y4],
              data.tagid5: [data.x5, data.y5], data.tagid6: [data.x6, data.y6],
              data.tagid7: [data.x7, data.y7], data.tagid8: [data.x8, data.y8]}

    # Assign where the robots belong in the system (shortest total distance required to travel to be in formation).
    simulation = False
    robot_to_node_assignment.assign_nodes(robots, goal_positions, simulation)

    # Move all the robots in the system to their desired goal.
    for i in range(0, len(robots)):
        robot = robots[i]

        # Convert the tags' positions to meters. If not able to detect the tag, use last information recieved.
        if camera.get(1) is not None:
            robot_position_back = vectors.multiply(camera.get(i * 2 + 1), 0.001)
            robot.set_position(robot_position_back)
        if camera.get(2) is not None:
            robot_position_front = vectors.multiply(camera.get(i * 2 + 2), 0.001)
            robot.set_front_position(robot_position_front)

        robot_position_front = robot.get_front_position()
        robot_position_back = robot.get_position()

        # Calculate orientation of robot, it should take a value from 0 to 2pi
        robot_orientation = math.atan2(robot_position_front[1] - robot_position_back[1],
                                       robot_position_front[0] - robot_position_back[0])
        if robot_orientation < 0:
            robot_orientation += 2 * math.pi

        # Set orientation of robot, so that it can be used later on when needed.
        robot.set_orientation(robot_orientation)

        # Find the specific robot's goal using information calculated in the node assignment algorithm.
        goal_position = goal_positions[robot.get_node_in_system()]

        # It is always a good idea to limit the speed when using real robots with risk of collisions.
        max_speed = 0.2
        move_robot_to_goal(robot, goal_position, max_speed)


# ------------------------------------------------------- PART 2 -------------------------------------------------------
# ----------------------------------- Calculations of the movement of the formation ------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function to move the formation. Subscribes to the leader separately as it has another
# callback function, the leader does not care at all about the other robots but instead
# just moves towards its goal. All robots in the system apart from the leader are defined
# as followers.

def move_formation_controller(robots, goal, distances):
    global at_goal
    at_goal = False
    global iterations
    iterations = 0

    # Make sure that the code at least runs one time, and if they are in position they will notify the system
    # themselves.
    robots[0].set_at_position(False)
    robots[1].set_at_position(False)
    robots[2].set_at_position(False)

    # The variables for the PID-controller have to be manually adjusted in order to make the robots moving as efficient
    # as possible. You have to define "efficient" yourself.
    # For the movement of the formation the followers have to consider the distance to both the other robots in the
    # system. Therefore the leader just uses one PID to approach its goal while the followers use two PID's each, one
    # for keeping the right distance from the leader and one for the other follower. Maybe it's beneficial to have a
    # higher gain for getting closer to the leader as it is moving on its own, while both of the followers try to get
    # closer to each other. This is totally up to you though.
    # Naming convention: pid_"ROBOT"_"TARGET".
    P = 0.7
    I = 1.3
    D = 0.0003
    pid_follower_1_follower = PID.PidController(P, D, I)
    pid_follower_1_leader = PID.PidController(P*1.2, D*1.05, I)
    pid_follower_2_follower = PID.PidController(P, D, I)
    pid_follower_2_leader = PID.PidController(P*1.2, D*1.05, I)
    pid_list = [pid_follower_1_leader, pid_follower_1_follower, pid_follower_2_leader, pid_follower_2_follower]

    # Find the shortest path for the leader to the goal. Set an area in which the robots move below. (Preferably equal
    # to the area the camera can see).
    minx = -2
    miny = -2
    maxx = 2
    maxy = 2
    path = find_path(goal, robots, minx, maxx, miny, maxy)

    # Subscribe to position data from the camera and send relevant information for movement of formation.
    subscriber = rospy.Subscriber("/position", Pos, move_formation_callback, (robots, path, distances, pid_list))

    # Make sure it keeps running until leader is at goal and the followers are in formation.
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            at_goal = True
            # When all robots are at the goal in formation, make sure they stop moving by adding a time delay during
            # which they can use the boolean at_goal to stop. Not really a problem since the robots stop moving after a
            # while anyways, but always better to be safe than sorry.
            time.sleep(1)
            break

    print "We're in formation at goal! "

    # Unsubscribe to the position data when done.
    subscriber.unregister()

    # Take a short break
    rospy.sleep(0.5)


# Callback function for the received data from the camera positioning system. Calculates the orientation of the robots
# and saves it along with the position in the respective robot object. Afterwards it takes action for each robot using
# information about if it is a leader or a follower. It uses Part 3 for the movement.

def move_formation_callback(data, args):
    camera = {data.tagid1: [data.x1, data.y1], data.tagid2: [data.x2, data.y2],
              data.tagid3: [data.x3, data.y3], data.tagid4: [data.x4, data.y4],
              data.tagid5: [data.x5, data.y5], data.tagid6: [data.x6, data.y6],
              data.tagid7: [data.x7, data.y7], data.tagid8: [data.x8, data.y8]}

    robots = args[0]
    goal = args[1]
    distances = args[2]
    pid_list = args[3]

    # Calculate the positions and orientations of the robots in the system and store them in their respective objects.
    for i in range(0, len(robots)):
        robot = robots[i]

        # Convert the tags' positions to meters. If not able to detect the tag, use last information recieved.
        if camera.get(1) is not None:
            robot_position_back = vectors.multiply(camera.get(i * 2 + 1), 0.001)
            robot.set_position(robot_position_back)
        if camera.get(2) is not None:
            robot_position_front = vectors.multiply(camera.get(i * 2 + 2), 0.001)
            robot.set_front_position(robot_position_front)

        robot_position_front = robot.get_front_position()
        robot_position_back = robot.get_position()

        # Calculate orientation of robot, it should take a value from 0 to 2pi
        robot_orientation = math.atan2(robot_position_front[1] - robot_position_back[1],
                                       robot_position_front[0] - robot_position_back[0])
        if robot_orientation < 0:
            robot_orientation += 2 * math.pi

        # Set orientation of robot, so that it can be used later on when needed.
        robot.set_orientation(robot_orientation)

    # Check if the robot is a leader or follower and take action accordingly.
    for i in range(0, len(robots)):

        max_speed_leader = 0.1
        max_speed_follower = 0.2

        if robots[i].get_node_in_system() == 0:
            move_leader(robots[i], goal, max_speed_leader)

        elif robots[i].get_node_in_system() == 1:
            pid_leader = pid_list[0]
            pid_follower = pid_list[1]
            move_follower(robots[i], robots, max_speed_follower, pid_leader, pid_follower, distances)

        elif robots[i].get_node_in_system() == 2:
            pid_leader = pid_list[2]
            pid_follower = pid_list[3]
            move_follower(robots[i], robots, max_speed_follower, pid_leader, pid_follower, distances)

        else:
            print "---WARNING--- THIS ROBOTS IS NEITHER LEADER NOR FOLLOWER. ---WARNING---"


# ------------------------------------------------------- PART 3 -------------------------------------------------------
# --------------------------------------------------- Moving a robot ---------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# A simple movement algorithm that moves the robot smoothly to a goal.

def move_robot_to_goal(robot, goal_position, max_speed):
    global at_goal
    # Position of robot in format [x,y]
    robot_position = robot.get_position()

    # Orientation of robot in radians from 0 to 2pi. Given in ACW direction from the positive x-axis.
    robot_orientation = robot.get_orientation()

    # The angle of the goal position given in radians in the same way as the orientation of the robot.
    goal_angle = angles.angle_between_points(goal_position, robot_position)

    # Target angle to aim for.
    target_angle = angles.angle_difference(goal_angle, robot_orientation)

    # Distance to the goal.
    distance = vectors.distance_points(goal_position, robot_position)

    # Spin the robot towards the desired orientation. In general a P-regulator should be enough.
    twist.angular.z = 0.5 * target_angle

    # Move the robot forward. The further away it is from the goal, as well as earlier error and predicted future error
    # by the PID is considered in the variable u. Also the robot will move by full speed when oriented correctly, but
    # slower the further away it is from its desired orientation given by target_angle.
    twist.linear.x = distance * ((math.pi - math.fabs(target_angle)) / math.pi)

    # If the robot is in position (within a margin), don't spin. This is due to the fact that the orientation it had
    # earlier should be good enough, and it might end up spinning in circles if it gets too close to the target.
    # Also let the robot know whether it is in position or not.
    tol = 0.1
    if distance < tol:
        robot.set_at_position(True)
        twist.angular.z = 0
        twist.linear.x = 0
    elif distance >= tol:
        robot.set_at_position(False)

    # If the robot is moving too fast, slow down please. It shouldn't be possible to get a negative speed but in case
    # that happens, just set the speed to 0.
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed
    elif twist.linear.x < 0:
        twist.linear.x = 0

    # If all the robots are at goal we have to stop moving of course.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    robot.pub.publish(twist)


# As above but moving the robot to points in a list, and proceeding to the next point when reached. When going to points
# in general it moves with its maximum speed scaled by being in the correct orientation, but uses a P regulator when
# going to the final point which is the goal.

def move_leader(robot, goal_list, max_speed):
    global at_goal
    global iterations
    goal_position = goal_list[iterations]
    # Position of robot in format [x,y]
    robot_position = robot.get_position()

    # Orientation of robot in radians from 0 to 2pi. Given in ACW direction from the positive x-axis.
    robot_orientation = robot.get_orientation()

    # The angle of the goal position given in radians in the same way as the orientation of the robot.
    goal_angle = angles.angle_between_points(goal_position, robot_position)

    # Target angle to aim for.
    target_angle = angles.angle_difference(goal_angle, robot_orientation)

    # Distance to the goal.
    distance = vectors.distance_points(goal_position, robot_position)

    # Spin the robot towards the desired orientation. In general a P-regulator should be enough.
    twist.angular.z = 0.5 * target_angle

    # Move the robot forward. The further away it is from the goal, as well as earlier error and predicted future error
    # by the PID is considered in the variable u. Also the robot will move by full speed when oriented correctly, but
    # slower the further away it is from its desired orientation given by target_angle.
    if iterations == len(goal_list) - 1:
        twist.linear.x = distance * ((math.pi - math.fabs(target_angle)) / math.pi)
    else:
        twist.linear.x = max_speed * ((math.pi - math.fabs(target_angle)) / math.pi)

    # If the robot is in position (within a margin), don't spin. This is due to the fact that the orientation it had
    # earlier should be good enough, and it might end up spinning in circles if it gets too close to the target.
    # Also let the robot know whether it is in position or not.
    tol = 0.2
    if distance < tol:
        if iterations == len(goal_list) - 1:
            robot.set_at_position(True)
            twist.angular.z = 0
            twist.linear.x = 0
        else:
            iterations += 1
    elif distance >= tol:
        robot.set_at_position(False)


    # If the robot is moving too fast, slow down please. It shouldn't be possible to get a negative speed but in case
    # that happens, just set the speed to 0.
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed
    elif twist.linear.x < 0:
        twist.linear.x = 0

    # If all the robots are at goal we have to stop moving of course.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    robot.pub.publish(twist)


# Function to move a follower which means that it adjusts the speed and direction for the robot considering the position
# of the other robots in the system as well as the desired distances in between them.

def move_follower(robot, robots, max_speed, pid_leader, pid_follower, distances):
    global at_goal

    # Find what the other robots are in the system.
    for i in range(0, len(robots)):
        if robots[i].get_node_in_system() == 0:
            leader = robots[i]
        elif (robots[i].get_node_in_system() != 0) and (robots[i] is not robot):
            follower = robots[i]

    # Desired distances to keep to the other robots. Leader and follower are either correctly assigned or something else
    # is terribly wrong. Shouldn't initialise them as a safety since that wouldn't allow you to identify the problem.
    desired_distance_leader = distances[leader.get_node_in_system()][robot.get_node_in_system()]
    desired_distance_follower = distances[follower.get_node_in_system()][robot.get_node_in_system()]

    # The robots' positions.
    robot_position = robot.get_position()
    follower_position = follower.get_position()
    leader_position = leader.get_position()

    # This follower's orientation
    robot_orientation = robot.get_orientation()

    # The orientation of the leader
    leader_orientation = leader.get_orientation()

    # Calculate the actual distances to the other robots.
    distance_leader = vectors.distance_points(leader_position, robot_position)
    distance_follower = vectors.distance_points(follower_position, robot_position)

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
    if u_leader != 0 and u_follower != 0:
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

    scale = (math.exp(-u) ** 2) * 2 / 3
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
    tol = 0.05
    if math.fabs(error_follower) < tol and math.fabs(error_leader) < tol:
        robot.set_at_position(True)
    else:
        robot.set_at_position(False)

    # Make sure the robot is not moving too fast.
    if twist.linear.x > max_speed:
        twist.linear.x = max_speed

    if u_leader < 0:
        twist.linear.x = 0.07

    # If all the robots are at goal we have to stop moving of course.
    if at_goal:
        twist.linear.x = 0
        twist.angular.z = 0

    # If the robots are colliding it would be a good thing to just stop before an accident happens.
    if distance_leader < 0.7 or distance_follower < 0.5:
        twist.linear.x = 0
        twist.angular.z = 0


    robot.pub.publish(twist)


# ------------------------------------------------------- Part 4 -------------------------------------------------------
# -------------------------------------------- Move robots into orientation --------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function to rotate the robots into a goal orientation defined by the one calling the function.

def robot_orientation(robots, goal_orientation):
    for i in range(0, len(robots)):
        robots[i].set_at_position(False)
    sub = rospy.Subscriber("/position", Pos, orientation_callback, (robots, goal_orientation))

    # Make sure it keeps running. Should terminate when robots are in the correct orientation.
    while not rospy.is_shutdown():
        if robots[0].get_at_position() and robots[1].get_at_position() and robots[2].get_at_position():
            break

    print "We're in the correct orientation! "
    sub.unregister()
    # Take a short break
    rospy.sleep(0.5)


# Callback function which uses positions from the camera system to rotate the robots into the correct orientation.

def orientation_callback(data, args):
    robots = args[0]
    goal_orientation = args[1]
    tol = 0.05  # Tolerance in radians for accepting the robot as being in the correct orientation.

    camera = {data.tagid1: [data.x1, data.y1], data.tagid2: [data.x2, data.y2],
              data.tagid3: [data.x3, data.y3], data.tagid4: [data.x4, data.y4],
              data.tagid5: [data.x5, data.y5], data.tagid6: [data.x6, data.y6],
              data.tagid7: [data.x7, data.y7], data.tagid8: [data.x8, data.y8]}

    for i in range(0, len(robots)):
        robot = robots[i]

        # If not able to detect the tag, use last information recieved.
        if camera.get(1) is not None:
            position_back = camera.get(i * 2 + 1)
            robot.set_position(position_back)
        if camera.get(2) is not None:
            position_front = camera.get(i * 2 + 2)
            robot.set_front_position(position_front)

        position_front = robot.get_front_position()
        position_back = robot.get_position()

        orientation = math.atan2(position_front[1] - position_back[1],
                                 position_front[0] - position_back[0])
        if orientation < 0:
            orientation += 2 * math.pi

        target_angle = angles.angle_difference(goal_orientation, orientation)

        if math.fabs(target_angle) > tol:
            twist.angular.z = 0.4 * target_angle
            robot.set_at_position(False)
        else:
            twist.angular.z = 0
            robot.set_at_position(True)

        robot.pub.publish(twist)


# ------------------------------------------------------- Part 5 -------------------------------------------------------
# ---------------------------------------------------- Path finding ----------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# Function that makes sure the position data is up to date, create a matrix representing the camera area and then calls
# the A* algorithm to find the shortest path to the goal from the leader in this matrix.

def find_path(goal, robots, minx, maxx, miny, maxy):
    global subscriber
    subscriber = rospy.Subscriber("/position", Pos, path_callback, robots)

    # Make sure we've collected the data by waiting a while (the subscriber will unsubscribe itself when done).
    time.sleep(1)

    # Set the robot closest to the target as leader.
    leader_assignment.set_leader(goal, robots, False)

    # Find the robots' positions.
    for i in range(0, len(robots)):
        if robots[i].get_node_in_system() == 0:
            leader_position = robots[i].get_position()
        elif robots[i].get_node_in_system() == 1:
            follower_1_position = robots[i].get_position()
        elif robots[i].get_node_in_system() == 2:
            follower_2_position = robots[i].get_position()

    # Now let's create a matrix representing the space in which the robots operate.
    # Number of intervals per meter.
    scale = 10
    nx = int((maxx - minx) * scale)
    ny = int((maxy - miny) * scale)
    # A matrix filled with zeros representing the area that the camera sees.
    matrix = [[0 for j in range(ny)] for i in range(nx)]

    # The start and goal positions represented in the matrix.
    start_matrix = (int((leader_position[0] - minx) * scale), int((leader_position[1] - miny) * scale))
    goal_matrix = (int((goal[0] - minx) * scale), int((goal[1] - miny) * scale))

    # Place followers in matrix and add some margins around them so that the leader at least not is aiming straight
    # towards them.
    follower_1_position_matrix = [int((follower_1_position[0] - minx) * scale),
                                  int((follower_1_position[1] - miny) * scale)]
    follower_2_position_matrix = [int((follower_2_position[0] - minx) * scale),
                                  int((follower_2_position[1] - miny) * scale)]

    # Set size of the followers so that they are represented correctly in the matrix. Number in meters, later scaled.
    width_follower = 0.7 * scale
    # Place ones in the matrix where the followers are + a region around them so that the leader will not move there.
    for i in range(nx):
        for j in range(ny):
            if vectors.distance_points(follower_1_position_matrix, [i,j]) <= width_follower or \
                            vectors.distance_points(follower_2_position_matrix, [i, j]) <= width_follower:
                matrix[i][j] = 1

    # Place obstacles in matrix and add some space around them so that they are not just one matrix element wide.
    global obstacles
    width_obstacle = 1 * scale
    if obstacles[0] != 0:
        obstacle_0_position_matrix = [int((obstacles[0][0] - minx) * scale),
                                      int((obstacles[0][1] - miny) * scale)]
        for i in range(nx):
            for j in range(ny):
                if vectors.distance_points(obstacle_0_position_matrix, [i,j]) <= width_obstacle:
                    matrix[i][j] = 1

    if obstacles[1] != 0:
        obstacle_1_position_matrix = [int((obstacles[1][0] - minx) * scale),
                                      int((obstacles[1][1] - miny) * scale)]
        for i in range(nx):
            for j in range(ny):
                if vectors.distance_points(obstacle_1_position_matrix, [i,j]) <= width_obstacle:
                    matrix[i][j] = 1

    # Find the shortest path using the A* algorithm
    cam_area = numpy.array(matrix)
    path = astar.astar(cam_area, goal_matrix, start_matrix)

    # Shift the path so that it corresponds to the actual area that is considered.
    if path is not False:
        for i in range(len(path)):
            path[i] = vectors.add(vectors.multiply(path[i], 1. / scale), [minx, miny])

    # It seems to be a good idea to not tell the robot to go to a point really close to it, so just cut out the first
    # 5 or so points. Also add the goal position at the end since it is not given from the A* algorithm.
    short_path = [0 for i in range(len(path) - 4)]
    for i in range(5, len(path)):
        short_path[i - 5] = path[i]
    short_path[len(short_path) - 1] = goal

    return short_path

# A callback function to store information about obstacles and the robots' positions. Does not really do anything.
def path_callback(data, robots):
    camera = {data.tagid1: [data.x1, data.y1], data.tagid2: [data.x2, data.y2],
              data.tagid3: [data.x3, data.y3], data.tagid4: [data.x4, data.y4],
              data.tagid5: [data.x5, data.y5], data.tagid6: [data.x6, data.y6],
              data.tagid7: [data.x7, data.y7], data.tagid8: [data.x8, data.y8]}

    if camera.get(1) is not None:
        robot_0_position = vectors.multiply(camera.get(1), 0.001)
        robots[0].set_position(robot_0_position)
    else:
        print "Can't see robot 0"
    if camera.get(3) is not None:
        robot_1_position = vectors.multiply(camera.get(3), 0.001)
        robots[1].set_position(robot_1_position)
    else:
        print "Can't see robot 1"
    if camera.get(5) is not None:
        robot_2_position = vectors.multiply(camera.get(5), 0.001)
        robots[2].set_position(robot_2_position)
    else:
        print "Can't see robot 2"

    global obstacles
    if camera.get(7) is not None:
        obstacle_0_position = vectors.multiply(camera.get(7), 0.001)
        obstacles[0] = obstacle_0_position
    else:
        obstacles[0] = 0
    if camera.get(8) is not None:
        obstacle_1_position = vectors.multiply(camera.get(8), 0.001)
        obstacles[1] = obstacle_1_position
    else:
        obstacles[1] = 0

    global subscriber
    subscriber.unregister()

