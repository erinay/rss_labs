#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

import tf
from rrt import RRT
import scipy.ndimage
from scipy.misc import imread, imsave

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        # self.odom_topic = 
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
    
        self.max_dist = 25
        self.start = None
        # self.init_set =W False

    def map_cb(self, msg):
        # rospy.loginfo('map_cb')
        obstacle_info = np.array(msg.data)
        self.map_resolution = msg.info.resolution
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_origin_pos = msg.info.origin.position
        map_quat = msg.info.origin.orientation
        quat_mat = [map_quat.x, map_quat.y, map_quat.z, map_quat.w]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(quat_mat)

        # Dilate & erode
        # erode = scipy.ndimage.binary_erosion(obstacle_info, iterations=1)
        dilated_data = scipy.ndimage.binary_dilation(obstacle_info, iterations=10) 
        copy_data = np.zeros_like(dilated_data, dtype=int)
        for ind, occup in enumerate(dilated_data):
            if occup:
                copy_data[ind]=100

        # rospy.loginfo(copy_data)

        self.data = copy_data.reshape((self.map_height, self.map_width))
        # imsave('/home/racecar/racecar_ws/src/path_planning/result.png', self.data)


        rot = tf.transformations.quaternion_matrix(quat_mat)
        self.rot_mat = np.array([rot[0,0:2],rot[1, 0:2]])
        quat_inv = tf.transformations.quaternion_inverse(quat_mat)
        rot_inv = tf.transformations.quaternion_matrix(quat_inv)
        self.rot_mat_inv = np.array([rot_inv[0,0:2],rot_inv[1, 0:2]])
        
    def pixel2Map(self, pixel):
        # Convert pixel to coordinate
        u = pixel[1]*self.map_resolution
        v = pixel[0]*self.map_resolution
        conv = np.array([u, v])-np.array([self.map_origin_pos.x, self.map_origin_pos.y])
        xy_map = np.dot(self.rot_mat_inv, conv)
        # x = np.matmul(self.tr_mat[0][:],np.array([[u],[v],[1]]))
        # y = np.matmul(self.tr_mat[1][:],np.array([[u],[v],[1]]))

        return (xy_map[0].astype(float),xy_map[1].astype(float))
    
    def map2Pixel(self,point):
        # Transformation *inverse *world fraself.map_origin_pos.xme 
        # x = point[0]
        # y = point[1]
        # pt = np.array([[x], [y]])
        
        conv = np.dot(self.rot_mat, point)+np.array([self.map_origin_pos.x, self.map_origin_pos.y])
        u = conv[0]/self.map_resolution
        v = conv[1]/self.map_resolution
        uact = np.round(u).astype(int)
        # rospy.loginfo(uact)
        vact = np.round(v).astype(int)

        return (np.asscalar(vact), np.asscalar(uact))

    def odom_cb(self, msg):
        # rospy.loginfo('odom callback')
        # Use Ground Truth Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        quat_mat = [quat.x, quat.y, quat.z, quat.w]
        theta = tf.transformations.euler_from_quaternion(quat_mat)
        self.start = (x, y)
        # pixel = self.map2Pixel(self.start)
        # rospy.loginfo(pixel)

    def goal_cb(self, msg):
        # Pose Stamped message
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        quat = msg.pose.orientation
        quat_mat = [quat.x, quat.y, quat.z, quat.w]
        goal_theta = tf.transformations.euler_from_quaternion(quat_mat)[2]
        
        self.goal = [goal_x, goal_y]

        rospy.loginfo('Set Goal Position')
        self.plan_path(self.start, self.goal, self.data)


    def plan_path(self, start_point, end_point, map_data):
        ## CODE FOR PATH PLANNING ##
        start = self.map2Pixel(start_point)
        goal = self.map2Pixel(end_point)
        bound = np.array([self.map_width, self.map_height])
        # rospy.loginfo(start)
        # rospy.loginfo(type(start))
        rrt_planning = RRT(start, goal, map_data, bound, self.max_dist, self.map_resolution, self.map_origin_pos)
        path = rrt_planning.path
        # map_path = []
        self.trajectory.clear()
        if path is not None:
            # Convert path to xy coordinates for robot
            for pt in path.path:
                xy_pt = self.pixel2Map(pt)
                # map_path.append(xy_pt)

                point = Point()
                point.x = xy_pt[0]
                point.y = xy_pt[1]
                self.trajectory.addPoint(point)

            # publish trajectory
            self.traj_pub.publish(self.trajectory.toPoseArray())
                    # visualize trajectory Markers
            self.trajectory.publish_viz()
            # for node in rrt_planning.final_gr:
            #     xy_pt = np.asarray(node)
        else:
            rospy.loginfo('no path')

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
