#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    DRIVE_TOPIC = rospy.get_param("safety_controller/drive_topic")
    SIDE = rospy.get_param("safety_controller/side")
    VELOCITY = rospy.get_param("safety_controller/velocity")
    DESIRED_DISTANCE = rospy.get_param("safety_controller/desired_distance")

    def __init__(self):

        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)
        rospy.Subscriber()
