from styx_msgs.msg import TrafficLight
import cv2


class TLClassifier(object):
    def __init__(self):
        # for the real car should probably do an actual nueral network. In the simulator the images are consistent enough we can just look for the color red
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        ## Gen lower mask (0-5) and upper mask (175-180) of RED
        mask1 = cv2.inRange(img_hsv, (0,50,20), (5,255,255))
        mask2 = cv2.inRange(img_hsv, (175,50,20), (180,255,255))

        ## Merge the mask and crop the red regions
        mask = cv2.bitwise_or(mask1, mask2)

        # red light sum:  798660
        # green light sum: 13515
        if mask.sum() > 100000:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
