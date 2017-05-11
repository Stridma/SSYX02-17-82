#!/usr/bin/env python

"""
This file contains functions that does calculations or operations
on vectors/points in 2D.
As of March 17 2017 it contains:
*Distance between two points.
*Length of a vector.
*Summation of two vectors.
As of April 10 2017 the following have been included to the functions above:
*Subtraction of two vectors.
*Normalisation of a vector.
*Multiplication of a vector by a scalar.
"""

import math


# Function that calculates the distance between two points, using the Pythagorean theorem.

def distance_points(pos1, pos2):
    return math.sqrt((pos1[1] - pos2[1]) ** 2 + (pos1[0] - pos2[0]) ** 2)


# Function that calculates the length of a vector, using the Pythagorean theorem.

def length(vector):
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2)


# Function that adds two vectors together, using regular algebra:
# A = [a1, a2], B = [b1, b2]
# => A + B = [a1 + b1, a2 + b2]

def add(vector1, vector2):
    return [vector1[0] + vector2[0], vector1[1] + vector2[1]]


# Function that subtracts two vectors from each other, using regular algebra:
# A = [a1, a2], B = [b1, b2]
# => A - B = [a1 - b1, a2 - b2]
def subtract(vector1, vector2):
    return [vector1[0] - vector2[0], vector1[1] - vector2[1]]


# Function to normalise a vector, e.g. length(normalise(vector)) = 1
def normalise(vector):
    len = length(vector)
    return [vector[0]/len, vector[1]/len]


# Function to multiply a vector by a scalar.
def multiply(vector, scalar):
    return [vector[0] * scalar, vector[1] * scalar]
