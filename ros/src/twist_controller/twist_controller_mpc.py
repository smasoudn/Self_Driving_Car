import rospy
from mpc.srv import *
import matplotlib.pyplot as plt
import numpy as np
import math


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        rospy.wait_for_service('mpc_service')
        self. mpc_control = rospy.ServiceProxy('mpc_service', mpc)
        plt.axis([-100, 100, -100, 100])
        axes = plt.gca()
        axes.set_ylim([-100, 100])

        axes.set_xlim([-100, 100])
        plt.ion()




    def control(self, ptsx, ptsy, px, py, psi, speed, steering, throttle):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.0
        brake = 0.5
        steering = 0.0
        try:
            result = self.mpc_control(ptsx, ptsy, px, py, psi, speed, steering, throttle)
            throttle = result.throttle
            steering = result.steering
            if throttle <= 0:
                brake = -throttle
                throttle = 0
            else:
                brake = 0


            # plot
            mpc_x = result.mpc_x
            mpc_y = result.mpc_y
            sz = len(mpc_x)

            plt.cla()
            wpx  = (np.array(ptsx) - px) * math.cos(psi) + (np.array(ptsy) -  py) * math.sin(psi)
            wpy  = -(np.array(ptsx) - px) * math.sin(psi) + (np.array(ptsy) -  py) * math.cos(psi)

            plt.plot(wpx, wpy, 'r')
            plt.plot(mpc_x, mpc_y, 'g')
            axes = plt.gca()
            axes.set_ylim([-50, 50])
            axes.set_xlim([-50, 50])
            plt.pause(0.01)



        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        rospy.logerr("python Client  {}  {}  {}".format(throttle, brake, steering))
        #brake = 1.0
        #throttle = 0.0

        return throttle, brake, steering
