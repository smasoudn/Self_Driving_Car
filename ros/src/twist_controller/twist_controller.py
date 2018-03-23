import rospy
from mpc.srv import *
import matplotlib.pyplot as plt
import numpy as np
import math


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd):
        # TODO: Implement
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.data_received = False
        self.p_error = 0
        self.i_error  = 0
        self.d_error = 0

        self.err = 0
        self.steps = 0
        self.cte = 0.0


    def update_error(self, cte):
        self.cte = cte
        if self.data_received == False:
            self.p_error = cte
            self.data_received = True

        self.d_error = cte - self.p_error;
        self.p_error = cte;
        self.i_error  += cte;
        self.err = self.err + cte if cte > 0  else  self.err - cte
        self.steps += 1


    def control(self):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer =  -self.kp * self.p_error - self.kd * self.d_error - self.ki * self.i_error
        #steer = -0.44724 if steer < -0.44724 else steer
        #steer = 0.44724 if steer > 0.44724 else steer
        steer = -1.0 if steer < -1 else steer
        steer = 1.0 if steer > 1 else steer
        rospy.logerr("cte: {}    steer:{}".format(self.cte, steer))
        return -steer


    def total_error(self):
        return err


    def get_steps(self):
        return self.steps
