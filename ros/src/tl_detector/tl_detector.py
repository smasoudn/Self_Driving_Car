#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import matplotlib.pyplot as plt
from scipy import spatial
import numpy as np
import math
import pickle


# constants
STATE_COUNT_THRESHOLD = 3
INF = 999999



class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.kdtree = []
        self.previous_dist_to_tl = None
        
        self.tl_idx = {}
        self.training_data = []
        self.d_cnt = 0
        self.set_cnt = 0

        rospy.spin()


    def pose_cb(self, msg):
        self.pose = msg



    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if self.waypoints and len(self.kdtree) == 0:
            self.fill_up_kdtree()



    def traffic_cb(self, msg):
        self.lights = msg.lights


    ######################################################################################################3
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if len(self.lights) == 0:
            return
        
        closest_light =  None
        stop_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # TODO find the closest visible traffic light (if one exists)
        if self.pose:
            current_car_idx = self.get_closest_waypoint(self.pose.pose)

            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                stop_line = Pose()
                stop_line.position.x = stop_line_positions[i][0]
                stop_line.position.y = stop_line_positions[i][1]
                tmp = self.get_closest_waypoint(stop_line)

                d = tmp - current_car_idx
                if d >=0 and d < diff:
                    diff = d
                    closest_light = light
                    stop_wp_idx = tmp

        if closest_light:
            state = self.get_light_state(closest_light)
            return stop_wp_idx, state

        return -1, TrafficLight.UNKNOWN



    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # TODO implement
        dist, idx = self.kdtree.query([pose.position.x, pose.position.y])
        wp_x = self.waypoints.waypoints[idx].pose.pose.position.x
        wp_y = self.waypoints.waypoints[idx].pose.pose.position.y

        car_x = self.pose.pose.position.x
        car_y = self.pose.pose.position.y
        car_theta = 2.0 * np.arccos(self.pose.pose.orientation.w)
        if car_theta > np.pi:
            car_theta = - (2.0 * np.pi - car_theta)

        heading = np.arctan2((wp_y - car_y), (wp_x - car_x))
        angle = abs(car_theta - heading)

        if angle > np.pi / 4:
            idx = (idx + 1) % len(self.waypoints.waypoints)

        return idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if not self.has_image:
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # cv2.imshow("camera", cv_image)
        # cv2.waitKey(1)

        # Get classification
        # return light.state

        return self.light_classifier.get_classification(cv_image)



    def fill_up_kdtree(self):
        wps = []
        for i, wp in enumerate(self.waypoints.waypoints):
            x = wp.pose.pose.position.x
            y = wp.pose.pose.position.y
            wps.append([x,y])
        self.kdtree = spatial.KDTree(np.array(wps))



    def distance(self, a, b):
        dx = (a.position.x - b.position.x)
        dy = (a.position.y - b.position.y)
        return np.sqrt(dx * dx + dy * dy)



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
