#!/usr/bin/env python2

import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector
from final_challenge_city.msg import SignLocationPixel

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/relative_sign_px", SignLocationPixel, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        (sign_prediction, bounding_box) = self.detector.predict(rgb_img)
        sign_location_pixel = SignLocationPixel()
        if not sign_prediction: ### Case where stop sign not detected
            sign_location_pixel.u = 0
            sign_location_pixel.v = 0
            sign_location_pixel.sign_detection = 0
            self.publisher.publish(sign_location_pixel)
        else:
            u = (bounding_box[0][0] + bounding_box[1][0]) / 2
            v = (bounding_box[0][1] + bounding_box[1][1]) / 2
            sign_location_pixel = SignLocationPixel()
            sign_location_pixel.u = u
            sign_location_pixel.v = v
            sign_location_pixel.sign_detection = 1
            self.publisher.publish(sign_location_pixel)


if __name__=="__main__":
    rospy.init_node("SignDetector", anonymous=True)
    detect = SignDetector()
    rospy.spin()
