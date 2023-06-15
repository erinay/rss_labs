#!/usr/bin/env python2

import numpy as np

import rospy
import math

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
#from visualization_tools import *
from final_challenge_race.msg import ConeLocation, ParkingError

class LineFollower():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    
    
    VELOCITY = 3.0 #rospy.get_param("final_challenge_race/velocity")
    Kp = 0.25#rospy.get_param("final_challenge_race/Kp")
    Kd = 0.12#0.85#.8 #rospy.get_param("final_challenge_race/Kd")
    #ERROR_TOPIC = rospy.get_param("final_challenge_race/error_topic")
    DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation" #rospy.get_param("~drive_topic")

    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation, self.callback)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error = rospy.Publisher("/error", Float32, queue_size=10)
        self.previous_angle_error=0
        self.previous_time = 0
        self.max_angle=0.65
        self.previous_x = None
        self.lookahead = 1.5# FILL IN #
        self.speed = 0.5# FILL IN #
        self.wheelbase_length = 0.8
        #self.line = rospy.Publisher("line", Marker ,queue_size=10)

    def control(self, error, curr_time):
        #error = cone_dist - self.parking_distance
        angle_error = error
        #angle_error = np.sign(error)*min(np.abs(self.max_angle), angle_error)
        #control_action = self.Kp * error + self.Kd * (error-self.previous_error)
        angle_action = self.Kp*angle_error + self.Kd*(angle_error-self.previous_angle_error)/(float(curr_time)-float(self.previous_time))
        self.previous_angle_error = angle_error
        self.previous_time = curr_time
        return angle_action

    def callback(self, msg):
        relative_x = -1*msg.x_pos
        self.relative_y = msg.y_pos
        rospy.loginfo([msg.x_pos, msg.y_pos])
        if self.previous_x is None:
            self.previous_x = [relative_x]*5
            #self.previous_x = [relative_x, relative_x, relative_x, relative_x, relative_x] 
        self.previous_x.pop(0)
        self.previous_x.append(relative_x)
        self.relative_x = sum(self.previous_x)/5
        #cone_dist = np.sqrt((self.relative_x**2 + self.relative_y**2))
        steer_angle = np.arctan2(self.relative_x,self.relative_y)
        # steer_angle = np.arctan2((2*self.wheelbase_length*sin(steer_angle)),self.lookahead)
        rospy.loginfo('steer angle')
        rospy.loginfo(steer_angle)
        now = rospy.get_rostime()
        curr_time =  now.nsecs
        angle_action = self.control(steer_angle, curr_time) 
        rospy.loginfo('angle_info')
        rospy.loginfo(angle_action)
        drive_stamp = AckermannDriveStamped()
        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp = rospy.Time.now()
        drive_stamp.header.frame_id = "line_follower"
        drive_stamp.drive.steering_angle = angle_action 
        # I think angle will be negative by axis convention
        # drive_stamp.drive.steering_angle_velocity = 0
        drive_stamp.drive.speed = self.VELOCITY
        #drive_stamp.drive.speed = self.VELOCITY - 2*angle_action/(0.34)
        
        drive_stamp.drive.acceleration = 0
        drive_stamp.drive.jerk = 0
        self.drive_pub.publish(drive_stamp)
        
        self.error.publish(Float32(self.relative_x))

        #self.drive_pub.publish(drive_stamp)
        #self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        #error_msg = Float32() 

        #error_msg.x_error = self.relative_x
        #error_msg.y_error = 0
        #error_msg.distance_error = 0
        self.error.publish(Float32(self.relative_x))
        #self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('LineFollower', anonymous=True)
        LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
