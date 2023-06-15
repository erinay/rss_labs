#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")*(-1)
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    Kp = rospy.get_param("wall_follower/Kp")
    Kd = rospy.get_param("wall_follower/Kd") * SIDE
    RELEVANT_RANGE = rospy.get_param("wall_follower/relevant_range") * DESIRED_DISTANCE
    ERROR_TOPIC = rospy.get_param("wall_follower/error_topic")
    MIN_RANGE = rospy.get_param("wall_follower/min_range") 

    def __init__(self):

        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)
        self.error = rospy.Publisher(self.ERROR_TOPIC, Float32, queue_size=10)
        self.line = rospy.Publisher("wall",Marker,queue_size=10)

    def scan_slice(self,scan,angles):
        half = int(len(scan.ranges)) /2
        if self.SIDE == -1:
            scan.ranges = scan.ranges[half:]
            sliced_angles = angles[half:]
        if self.SIDE == 1:
            scan.ranges = scan.ranges[:half]
            sliced_angles = angles[:half]

        return scan,sliced_angles

    def datapoint_angles(self,scan):
        return np.arange(scan.angle_min,scan.angle_max,scan.angle_increment)


    def find_the_wall(self,sliced_range,sliced_angles):
        x_dist_ranges_temp = np.cos(sliced_angles)*sliced_range
        y_dist_ranges_temp = np.sin(sliced_angles)*sliced_range

        ### get rid of edge cases. we only want scans that are close-ish to a wall
        long_values_index = np.where(abs(y_dist_ranges_temp) > self.RELEVANT_RANGE) ## 3.5 p
        x_dist_ranges1 = np.delete(x_dist_ranges_temp,long_values_index)
        y_dist_ranges1 = np.delete(y_dist_ranges_temp,long_values_index)
      #  rospy.loginfo(y_dist_ranges_temp)
        short_values_index = np.where(abs(y_dist_ranges1) < self.MIN_RANGE) 
        x_dist_ranges = np.delete(x_dist_ranges1, short_values_index)
        y_dist_ranges = np.delete(y_dist_ranges1, short_values_index)

        [theta, wall_distance] = np.polyfit(x_dist_ranges,y_dist_ranges,1)
        xmin = min(x_dist_ranges)
        xmax = max(x_dist_ranges)
        x = np.arange(xmin,xmax,.1)
        y = theta*x + wall_distance
        VisualizationTools.plot_line(x,y,self.line,frame = "/laser")
        return theta, wall_distance

    def control(self,theta,wall_distance):
        error = self.DESIRED_DISTANCE - abs(wall_distance)
        self.error.publish(Float32(error))
        control_action = self.SIDE * self.Kp * error + self.SIDE * self.Kd * self.VELOCITY * theta
        return control_action


    def callback(self,scan):
        angles = self.datapoint_angles(scan)
        sliced_scan,sliced_angles = self.scan_slice(scan,angles) # uses either the left or right side of the data
        reg = self.find_the_wall(sliced_scan.ranges,sliced_angles)
        (theta,wall_distance) = (reg[0],reg[1])
        control_action = self.control(theta,wall_distance)
        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp = rospy.Time.now()
        drive_stamp.header.frame_id = "wall_follower"
        drive_stamp.drive.steering_angle = control_action
        drive_stamp.drive.steering_angle_velocity = 0
        drive_stamp.drive.speed = self.VELOCITY
        drive_stamp.drive.acceleration = 0
        drive_stamp.drive.jerk = 0
        self.pub.publish(drive_stamp)



if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
