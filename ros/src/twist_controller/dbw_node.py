#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math
import numpy as np

from twist_controller import Controller

from std_msgs.msg import Bool
from styx_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
Kp = 0.607900;     #  Causes the car  react to sharp turns faster but makes it oscillats and unstable
Kd =  1.640951;     #  Prevents car from oscillating
Ki = .0001;  #  Reduce the biase gradually
    
    
class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.steer_controller = Controller(Kp, Ki, Kd)
        self.speed_controller = Controller(Kp, Ki, Kd)
        self.twist_cmd = None
        self.waypoints = None
        self.dbw_enabled = None
        self.cte = 0.0

        self.ptsx = []
        self.ptsy = [] 
        self.px = 0.0
        self.py = 0.0
        self.psi = 0.0 
        self.speed = 0.0 
        self.steering = 0.0 
        self.throttle = 0.0
        
        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/vehicle/throttle_report', Float32 , self.throttle_report_cb)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.steering_report_cb)

        self.loop()

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg.twist

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def waypoints_cb(self, msg):
        self.ptsx = []
        self.ptsy = []
        for i in range(len(msg.waypoints)):
            self.ptsx.append(msg.waypoints[i].pose.pose.position.x)
            self.ptsy.append(msg.waypoints[i].pose.pose.position.y)
        self.waypoints = msg.waypoints

    def current_pose_cb(self, msg):
        self.pose = msg.pose
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
        self.psi =  2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        #rospy.logerr("x: {}  y: {}  psi: {}".format(self.px, self.py, self.psi))

    def throttle_report_cb(self, msg):
        self.throttle = msg.data


    def steering_report_cb(self, msg):
        self.steering = msg.steering_wheel_angle
        self.speed = msg.speed

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if self.dbw_enabled:
                self.get_cte()
                self.steer_controller.update_error(self.cte);
                steering = self.steer_controller.control();          
                throttle = 0.2
                brake = 0.0
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


    
    def get_cte(self):
        quaternion = [self.pose.orientation.x,
                  self.pose.orientation.y,
                  self.pose.orientation.z,
                  self.pose.orientation.w]
        _, _, angle = euler_from_quaternion(quaternion)


        
        heading_waypoints_x = []
        heading_waypoints_y = []

        for i in range(20):
            translated_x = self.waypoints[i].pose.pose.position.x - self.pose.position.x
            translated_y = self.waypoints[i].pose.pose.position.y - self.pose.position.y

            heading_waypoints_x.append(translated_x * math.cos(0 - angle) - translated_y * math.sin(0 - angle))
            heading_waypoints_y.append(translated_x * math.sin(0 - angle) + translated_y * math.cos(0 - angle))

        fit = np.poly1d(np.polyfit(heading_waypoints_x, heading_waypoints_y, 2))
        self.cte = fit(2)


if __name__ == '__main__':
    DBWNode()
