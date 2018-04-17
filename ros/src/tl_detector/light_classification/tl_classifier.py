from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

RED_THRESHOLD = 200
GREEN_THRESHOLD = 100
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        b, g, r = cv2.split(image)
        r[r <= RED_THRESHOLD] = 0
        r[r > RED_THRESHOLD] = 255
        g[g > GREEN_THRESHOLD] = 255
        g[g <= GREEN_THRESHOLD] = 0
        g = 255 - g

        mask = np.array(r) * (np.array(g) / 255)
        num_red = np.sum(mask / 255)
        mask = cv2.merge((mask, 0 * r, 0 * r))

        if num_red >= 40:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
