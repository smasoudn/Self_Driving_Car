import rospy
from mpc.srv import *

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        rospy.wait_for_service('mpc_server')
        self. mpc_control = rospy.ServiceProxy('mpc_server', mpc)


    def control(self, ptsx, ptsy, px, py, psi, speed, steering, throttle):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        try:
            result = self.mpc_control(ptsx, ptsy, px, py, psi, speed, steering, throttle)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        throttle = result.throttle
        steering = result.steering
        brake = 0 if throttle > 0 else 1

        return throttle, brake, steering
