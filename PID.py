#!/usr/bin/env python

"""
A PID-controller that could be used for anything (more or less). Has no wind-up implemented for the integral term.
"""


import time


class PidController:
    def __init__(self, K_p, K_i, K_d):
        self.time_last = time.time()
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        # Integral term
        self.int = 0
        self.last_error = 0

    def pid(self, error):
        time_now = time.time()
        time_d = time_now - self.time_last

        # Save errors in the integral term
        self.int += time_d * error

        # Predict the future by the current change rate
        der = (error - self.last_error) / time_d

        # Calculate a control command
        u = self.K_p * error + self.int * self.K_i + der * self.K_d

        self.time_last = time_now
        self.last_error = error

        return u