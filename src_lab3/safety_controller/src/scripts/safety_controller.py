#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


class SafetyController:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    DRIVE_COMMANDS = rospy.get_param("safety_controller/drive_output")
    SAFETY_TOPIC = rospy.get_param("safety_controller/safety_topic")
    stop_distance = rospy.get_param("safety_controller/stop_distance")  # in meters
    worry_distance = rospy.get_param("safety_controller/worry_distance")
    worry_speed = rospy.get_param("safety_controller/worry_speed")
    relevant_indices = rospy.get_param("safety_controller/relevant_indices")
    point_threshold = rospy.get_param("safety_controller/point_threshold")
    VEL_TOPIC = 'velocity'

    def __init__(self):

        self.pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size=10)
        self.steering_angle = 0
        self.velocity = 0
        rospy.Subscriber(self.DRIVE_COMMANDS, AckermannDriveStamped, self.callback_drive)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.vel = rospy.Publisher(self.VEL_TOPIC, Float32, queue_size=10)

    def scan_slice(self, ranges):
        """
        Slice the data into a slice that our robot could come in contact with
        """
        front_facing = int(len(ranges) / 2)
        relevent_scan = ranges[front_facing - self.relevant_indices: front_facing + self.relevant_indices]
        return relevent_scan

    # I think we need to subscribe to racecar
    def callback_drive(self, data):
        self.steering_angle = data.drive.steering_angle
        self.velocity = data.drive.steering_angle
        self.vel.publish(Float32(self.velocity))

    def callback(self, LaserScan):
        # Subscriber for Laser scan data

        racecar = AckermannDriveStamped()
        angle_min = LaserScan.angle_min
        angle_max = LaserScan.angle_max
        angle_increment = LaserScan.angle_increment

        # Get current steering angle and publish to racecar msg
        steering_angle = self.steering_angle
        velocity = self.velocity

        ranges = LaserScan.ranges

        relevent_scan = self.scan_slice(ranges)

        stop_violations = 0
        worry_violations = 0
        for data_point in relevent_scan:
            if data_point < self.stop_distance:
                # Houston, we have a problem
                stop_violations += 1
            elif self.worry_distance > data_point > self.stop_distance:
                print("enter worry loop")
                worry_violations += 1

            if stop_violations >= self.point_threshold:
                print("stop")
                racecar.drive.speed = 0
                self.pub.publish(racecar)
                break
            elif worry_violations >= self.point_threshold:
                print("worry")
                racecar.drive.speed = self.worry_speed
                self.pub.publish(racecar)
                

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()
