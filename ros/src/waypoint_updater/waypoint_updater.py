#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32

import math
from scipy import spatial
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_kdtree =  None
        self.waypoints = None
        self.current_pose = None
        self.current_pose_idx = None
        self.stop_idx = None
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))

        self.loop()


    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints_kdtree:
                idx = self.closest_point()
                self.publish(idx)
            rate.sleep()



    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose



    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if not self.waypoints_kdtree:
            self.waypoints = waypoints.waypoints
            wps = []
            for i, wp in enumerate(self.waypoints):
                x = wp.pose.pose.position.x
                y = wp.pose.pose.position.y
                wps.append([x, y])
            self.waypoints_kdtree = spatial.KDTree(np.array(wps))


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_idx =  msg.data


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def closest_point(self):
        _, idx = self.waypoints_kdtree.query([self.current_pose.position.x, self.current_pose.position.y], 1)

        closest_wp = np.array([self.waypoints[idx].pose.pose.position.x, self.waypoints[idx].pose.pose.position.y])
        previous_wp = np.array([self.waypoints[idx-1].pose.pose.position.x, self.waypoints[idx-1].pose.pose.position.y])
        current_pose = np.array([self.current_pose.position.x, self.current_pose.position.y])

        val = np.dot(closest_wp-previous_wp, current_pose-closest_wp)
        if val > 0:
            idx = (idx + 1) % len(self.waypoints)

        return idx

    
    
    def publish(self,  closest_idx):
        lane = self.velocity_profile(closest_idx)
        self.final_waypoints_pub.publish(lane)


    def velocity_profile(self, closest_idx):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.waypoints[closest_idx:closest_idx + LOOKAHEAD_WPS]

        if self.stop_idx > -1:
            if (self.stop_idx - closest_idx) >= LOOKAHEAD_WPS:
                return lane

            lane.waypoints = self.decelerate_wps(lane.waypoints, closest_idx)
            rospy.logerr(">>>>>>>>>>>>  {}    {}    <<<<<<<<<<<<".format(self.get_waypoint_velocity(self.waypoints[closest_idx]),
                                                                     self.get_waypoint_velocity(lane.waypoints[0])))
        return lane


    def decelerate_wps(self, waypoints, closest_idx):
        temp = []

        stop_index = max(self.stop_idx - closest_idx - 2, 0)

        n = int(self.stop_idx) - int(closest_idx) - 2
        n = max(1, n)
        c = 0
        for i, wp in enumerate(waypoints):
            dist = self.distance(waypoints, i, stop_index)
            velocity = 0.3 * dist
            if velocity < 0.5:
                velocity = 0.0
            p = Waypoint()
            p.pose = wp.pose
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)

            temp.append(p)
        return  temp



    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)




if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
