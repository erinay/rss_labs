#!/usr/bin/env python

import rospy
import numpy as np
import math

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .1 # .75 meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.Kp = 1.5
        self.Kd = .4
        self.previous_angle_error = 0
        self.previous_time = 0
        self.previous_dist_error = 0

    def control(self, cone_dist, steer_angle, curr_time):
        #error = cone_dist - self.parking_distance
        angle_error = steer_angle 
        dist_error = cone_dist-self.parking_distance
        #control_action = self.Kp * error + self.Kd * (error-self.previous_error)
        angle_action = self.Kp*angle_error + self.Kd*(angle_error-self.previous_angle_error)/(float(curr_time)-float(self.previous_time))
        dist_action = self.Kp*dist_error + self.Kd*(dist_error-self.previous_dist_error)/(float(curr_time)-float(self.previous_time))
        self.previous_angle_error = angle_error
        self.previous_dist_error = dist_error
        self.previous_time = curr_time
        return angle_action, dist_action

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        cone_dist = np.sqrt((self.relative_x**2 + self.relative_y**2))
        steer_angle = math.atan(self.relative_y/self.relative_x)

        now = rospy.get_rostime()
        curr_time =  now.nsecs
        angle_action, dist_action = self.control(cone_dist, steer_angle, curr_time) 
        drive_cmd = AckermannDriveStamped()
        
        drive_cmd.header.frame_id = "parking controller"
        
        
        if cone_dist > self.parking_distance+.1:
            #if abs(angle_action) > 0.34*math.pi:
             #   drive_cmd.drive.steering_angle = 0.34*math.pi
            #else:
            
            #drive_cmd.drive.steering_angle = angle_action
            
            #drive_cmd.drive.steering_angle_velocity = 0
            #scaled_speed = 1
            #if cone_dist < self.parking_distance*2:
            #    scaled_speed = (cone_dist-self.parking_distance)/self.parking_distance
            drive_cmd.drive.steering_angle = steer_angle
            drive_cmd.drive.speed = min(1, dist_action)
            drive_cmd.drive.acceleration = 0
            drive_cmd.drive.jerk = 0
        elif cone_dist < self.parking_distance-.1:
            drive_cmd.drive.steering_angle = -steer_angle
            drive_cmd.drive.speed = -.3
        #else:
            #drive_cmd.drive.steering_angle = 0
            #drive_cmd.drive.speed = 0


        #turn in a circle to find the cone?
        #if (cone_dist <= 0 or cone_dist > self.parking_distance*6): #and self.relative_x < 0:
         #   rospy.loginfo("checking")
          #  drive_cmd.drive.steering_angle = 0.34*math.pi
           # drive_cmd.drive.speed = .5

        rospy.loginfo("steering from PD %s", angle_action)
        rospy.loginfo("                                          steering from atan %s", steer_angle)

        # if cone_dist > self.parking_distance:
        #     scaled_speed = 1
        #     if cone_dist < self.parking_distance*2:
        #         scaled_speed = (cone_dist-self.parking_distance)/self.parking_distance
        #     drive_cmd.drive.steering_angle = steer_angle
        #     drive_cmd.drive.speed = max(.5, scaled_speed)
        # elif cone_dist < self.parking_distance-.1:
        #     drive_cmd.drive.steering_angle = 0
        #     drive_cmd.drive.speed = -.3
        # else:
        #     drive_cmd.drive.steering_angle = 0
        #     drive_cmd.drive.speed = 0


        

        

        

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error = np.sqrt((self.relative_x**2 + self.relative_y**2))
        error_msg.distance_error = error
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
