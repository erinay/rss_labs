#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from final_challenge_race.msg import ConeLocation

from std_msgs.msg import Float32

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class LineDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone", ConeLocation, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        dimensions = image.shape
        width = dimensions[1]
        height = dimensions[0]
        lookahead_upper =int( height*6.5/12)
        lookahead_lower = height*11/12
        image_l = cv2.rectangle(image, (0,0), (width, lookahead_upper), (0,0,0), -1)
        image = cv2.rectangle(image_l, (0,lookahead_lower ), (width, height), (0,0,0), -1)
        #image = cv2.rectangle(image_u, (int(width*0.35), lookahead_upper), (int(width*0.65),lookahead_lower), (0,0,0), -1)
        #triangle1 = np.array([[(0, lookahead_upper), (0, int(height*0.75)), (int(width*0.10), lookahead_upper)]])
        #triangle2 = np.array([[(width, lookahead_upper), (width, int(height*0.75)), (int(width*0.90), lookahead_upper)]])
        #img = cv2.fillPoly(image, pts=[triangle1], color=(0,0,0))
        #image = cv2.fillPoly(img, pts=[triangle2], color=(0,0,0))
                
        #points = np.array([[], [], []], dtype=np.int32)
        #debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")

        left_line, right_line, goal_pos = cd_color_segmentation(image) #publish this pixel (u, v), what points are needed?
        msg = ConeLocation()
        msg.x_pos = goal_pos[0]
        msg.y_pos = height#goal_pos[1]
        self.cone_pub.publish(msg)


        inter_x = goal_pos[0]+width/2

        rospy.loginfo(goal_pos[1])
        image_c = cv2.circle(image_l, center=(inter_x, goal_pos[1]), radius=5, color=(0,255,0), thickness=-1)
        cv2.line(image_c, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0,0,255), 3, cv2.LINE_AA)
        cv2.line(image_c, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0,0,255), 3, cv2.LINE_AA)
        debug_msg = self.bridge.cv2_to_imgmsg(image_c, "bgr8")
        #rospy.loginfo('publishing debug image')
        #rospy.loginfo(left_line)
        #rospy.loginfo(right_line)
        #376,672rospy.loginfo([height, width])
        self.debug_pub.publish(debug_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        LineDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
