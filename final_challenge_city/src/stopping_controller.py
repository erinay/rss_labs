#!/usr/bin/env python

import rospy
import numpy as np
import math

from final_challenge_city.msg import SignLocation, StopDriveCmd
from ackermann_msgs.msg import AckermannDriveStamped


class StoppingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    def __init__(self):
        rospy.Subscriber("/relative_sign", SignLocation,
                         self.relative_sign_callback)
        self.drive_pub = rospy.Publisher("/stopping_controller",
                                         StopDriveCmd, queue_size=10)

        self.stopping_distance = 1.0
        self.relative_x = 0
        self.relative_y = 0
        self.detector = 0
        self.Kp = 1.5
        self.Kd = .4
        self.previous_angle_error = 0
        self.previous_time = 0
        self.previous_dist_error = 0


    def control(self, sign_dist, curr_time):

        dist_error = sign_dist - self.parking_distance
        dist_action = self.Kp * dist_error + self.Kd * (dist_error - self.previous_dist_error) / (
                    float(curr_time) - float(self.previous_time))
        self.previous_dist_error = dist_error
        self.previous_time = curr_time
        return dist_action

    def relative_sign_callback(self, sign_msg):
        self.relative_x = sign_msg.rel_x
        self.relative_y = sign_msg.rel_y
        self.detector = sign_msg.sign_detection
        sign_dist_mag = np.sqrt((self.relative_x ** 2 + self.relative_y ** 2))
        steer_angle = np.arctan2(self.relative_y, self.relative_x)
        sign_dist_x = np.cos(steer_angle) * sign_dist_mag
        if self.detector and sign_dist_x < self.stopping_distance:
            rospy.loginfo("Stop Sign Detected")
            stop_drive_command = StopDriveCmd()
            stop_drive_command.steering_angle = 0
            stop_drive_command.speed = 0
            stop_drive_command.sign_detection = 1
            self.drive_pub.publish(stop_drive_command)
        else:
            rospy.loginfo("Sign NOT Detected")
            stop_drive_command = StopDriveCmd()
            stop_drive_command.steering_angle = 0
            stop_drive_command.speed = 0
            stop_drive_command.sign_detection = 0
            self.drive_pub.publish(stop_drive_command)




if __name__ == '__main__':
    try:
        rospy.init_node('StoppingController', anonymous=True)
        StoppingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
