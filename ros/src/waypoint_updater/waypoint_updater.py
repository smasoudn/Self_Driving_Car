#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32

import math

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
        self.velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity'))
        self.waypoints = None
        self.current_pose = None
        self.current_pose_idx = None

        rospy.spin()

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
        if self.waypoints is None:
            return
        idx = self.closestPoint()
        self.current_pose_idx = idx
        
        next_points = []
        for i in range(LOOKAHEAD_WPS):
            next_points.append(self.waypoints[(i + idx)%len(self.waypoints)])
        
        self.publish(next_points)



    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data > -1:
            current_velocity = self.get_waypoint_velocity(self.waypoints[self.current_pose_idx])
            if msg.data >= self.current_pose_idx:
                n = int(msg.data) - int(self.current_pose_idx)
                if n == 0:
                    self.set_waypoint_velocity(self.waypoints, self.current_pose_idx, current_velocity)
                    return
                c = 0
                for idx in range(self.current_pose_idx, msg.data + 1):
                    velocity = current_velocity - c * current_velocity / n
                    self.set_waypoint_velocity(self.waypoints, idx, velocity)
                    c += 1

            else:
                n = len(self.waypoints) - int(self.current_pose_idx) + msg.data
                if n == 0:
                    self.set_waypoint_velocity(self.waypoints, self.current_pose_idx, current_velocity)
                    return
                c = 0
                for idx in range(self.current_pose_idx, len(self.waypoints)):
                    velocity = current_velocity - c * current_velocity / n
                    self.set_waypoint_velocity(self.waypoints, idx, velocity)
                    c += 1
                for idx in range(0, msg.data + 1):
                    velocity = current_velocity - c * current_velocity / n
                    self.set_waypoint_velocity(self.waypoints, idx, velocity)
                    c += 1
        else:
            self.set_waypoint_velocity(self.waypoints, self.current_pose_idx, self.velocity)

        #rospy.logerr("===  pose: {}    TL: {}".format(self.current_pose_idx, msg))





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


    def closestPoint(self):
        dist = lambda a, b: math.sqrt((a.pose.pose.position.x-b.position.x)**2 + 
                                      (a.pose.pose.position.y-b.position.y)**2  + 
                                      (a.pose.pose.position.z-b.position.z)**2)
        
        #rospy.logerr(len(self.waypoints))
        min_len = dist(self.waypoints[0], self.current_pose)
        idx = 0
        for i in range(len(self.waypoints)):
            distance = dist(self.waypoints[i], self.current_pose)
            if distance < min_len:
                min_len = distance
                idx = i
        return idx

    
    
    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)




if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
