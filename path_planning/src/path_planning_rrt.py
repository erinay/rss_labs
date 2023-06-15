#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

import tf
# from rrt import RRT
import scipy.ndimage
from scipy.misc import imread, imsave
from graph import *


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
        self.node_pub = rospy.Publisher('node', Marker, queue_size=10)
        self.box_pub = rospy.Publisher('collison_box', PoseArray, queue_size=10)
    
        self.target_range = 10
        self.max_dist = 10
        self.start = None
        # self.init_set =W False


    def map_cb(self, msg):
        rospy.loginfo('map_cb')
        obstacle_info = np.array(msg.data)
        self.map_resolution = msg.info.resolution
        self.map_height = msg.info.height
        self.map_width = msg.info.width
        self.map_origin_pos = msg.info.origin.position
        map_quat = msg.info.origin.orientation
        quat_mat = [map_quat.x, map_quat.y, map_quat.z, map_quat.w]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(quat_mat)

        # Dilate & erode
        dilated_data = scipy.ndimage.binary_dilation(obstacle_info, iterations=5) 
        copy_data = np.zeros_like(dilated_data, dtype=int)
        for ind, occup in enumerate(dilated_data):
            if occup:
                copy_data[ind]=100

        self.data = copy_data.reshape((self.map_height, self.map_width))
        # imsave('/home/racecar/racecar_ws/src/path_planning/result.png', self.data)


        self.rot = np.array(
            [[np.cos(self.yaw), -np.sin(self.yaw), 0],
            [np.sin(self.yaw), np.cos(self.yaw), 0],
            [0,0,1]]
        )
        
    def pixel2Map(self, pixel):
        # Convert pixel to coordinate
        u = pixel[0]
        v = pixel[1]
        # u = pixel[0]*self.map_resolution
        # v = pixel[1]*self.map_resolution
        conv = np.array([u, v, 0])#-np.array([self.map_origin_pos.x, self.map_origin_pos.y])
        xy_map = np.matmul(self.rot, conv)

        xact = xy_map[0]*self.map_resolution+self.map_origin_pos.x
        yact = xy_map[1]*self.map_resolution+self.map_origin_pos.y

        return (xact.astype(float),yact.astype(float))
    
    def map2Pixel(self,point):
        # Transformation *inverse *world fraself.map_origin_pos.xme 
        x = (point[0] - self.map_origin_pos.x)/self.map_resolution
        y = (point[1] - self.map_origin_pos.y)/self.map_resolution
        pt = np.array([x,y,0])
        
        conv = np.matmul(self.rot, pt)
        u = conv[0]
        v = conv[1]
        uact = np.round(u).astype(int)
        vact = np.round(v).astype(int)

        return (np.asscalar(uact), np.asscalar(vact))


    def odom_cb(self, msg):
        # rospy.loginfo('odom callback')
        # Use Ground Truth Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        quat_mat = [quat.x, quat.y, quat.z, quat.w]
        theta = tf.transformations.euler_from_quaternion(quat_mat)
        self.start = (x, y)

    def goal_cb(self, msg):
        # Pose Stamped message
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        quat = msg.pose.orientation
        quat_mat = [quat.x, quat.y, quat.z, quat.w]
        goal_theta = tf.transformations.euler_from_quaternion(quat_mat)[2]
        
        self.goal = [goal_x, goal_y]

        rospy.loginfo('Set Goal Position')
        self.plan_path(self.start, self.goal)


    def plan_path(self, start_point, end_point):
        ## CODE FOR PATH PLANNING ##

        start = self.map2Pixel(start_point)
        self.end_pixel = self.map2Pixel(end_point)
        self.bound = np.array([self.map_width, self.map_height])
        gr = Graph()
        gr.add_node(start)
        reached = (self.end_pixel==start)
        searchNodes = [SearchNode(start)]
        self.count = 0        
        parent=None


        # # # See if using list is faster
        # tree_nodes = []
        # tree_parent = []
        # tree_nodes.append(start)
        point = Marker()
        point.type=2
        point.color.r = 1.0
        point.color.a = 1.0
        point.header.frame_id = '/map'
        # Set the scale of the markers
        point.scale.x = 2
        point.scale.y = 2
        point.scale.z = 1.0

        
        # while(not reached):
        while(not reached and self.count<=5000):
            rospy.loginfo(self.count)
            rand_pt = self.random_path()
            xnear = gr.nearest(rand_pt)
            new_x = self.step(xnear, rand_pt, self.max_dist) #steering
            xy_pt = self.pixel2Map(new_x)
            # map_path.append(xy_pt)

            
            point.pose.position.x = xy_pt[0]
            point.pose.position.y = xy_pt[1]
            point.pose.position.z = 0
            point.pose.orientation.x = 0
            point.pose.orientation.y = 0
            point.pose.orientation.z = 0
            point.pose.orientation.w = 1
            self.node_pub.publish(point)

            # Check path collision
            if not self.collision(xnear, new_x):
                rospy.loginfo('no collision')
                gr.add_edge(xnear, new_x)
                # Find parent node to create SearchNode for Pathing
                for searchnode in searchNodes:
                    # If existing search node states is the same as the closest node
                    # Make new Search node instance with previous search node as the parent
                    if(searchnode._state == xnear):
                        parent = searchnode
                        break
                newnode = SearchNode(new_x, parent)
                searchNodes.append(newnode)
                    
                # check if point exists in goal
                g = np.asarray(self.end_pixel)
                diff = np.linalg.norm(g-new_x)
                if (diff < 10):
                    goalnode = SearchNode(self.end_pixel, newnode)
                    searchNodes.append(goalnode)
                    reached=True
                else: 
                    self.count += 1
            else:
                # rospy.loginfo('collision')
                self.count+=1
        path = Path(searchNodes[-1]) 
        # self.final_gr = gr._nodes

        # path = rrt_planning.path
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

    def random_path(self):
        # Set random node
        if (self.count%300==0):
            x = self.end_pixel[0]
            y = self.end_pixel[1]
        else:
            #place initial check for a valid point
            collide = True
            while(collide):
                y = np.random.randint(0, self.bound[1]) #height
                x = np.random.randint(0,self.bound[0]) #width
                map_data = self.data[y][x]
                if(map_data==0):
                    collide=False
        return [x, y]

    def step(self, xnear, xnew, step):
        x1 = np.asarray(xnear)
        x2 = np.asarray(xnew)
        dist = np.linalg.norm(x1-x2)
        xnew = xnear
        if(dist>self.max_dist):
            diff = x2-x1
            # Unit step
            dx = diff[0]/dist
            dy = diff[1]/dist

            theta = np.arctan2(diff[1], diff[0])
            x_step = np.cos(theta)
            y_step = np.sin(theta)
            
            x = xnear[0]+x_step*step
            y = xnear[1]+y_step*step
            xnew = (x,y)

        return(xnew)

    def collision(self, xnear, xnew):

        ##AABB collision checking
       
        xstart = np.round(np.asarray(xnear)).astype(int)
        xend = np.round(np.asarray(xnew)).astype(int)

        if xstart[0]<xend[0]:
            mapx_min = xstart[0].astype(int)
            mapx_max = xend[0].astype(int)+1

        else:
            mapx_min = xend[0].astype(int)
            mapx_max = xstart[0].astype(int)+1
        if xstart[1]<xend[1]:
            mapy_min = xstart[1].astype(int)
            mapy_max = xend[1].astype(int)+1

        else:
            mapy_min = xend[1].astype(int)
            mapy_max = xstart[1].astype(int)+1
        
        # rospy.loginfo([mapy_min, mapy_max])
        # rospy.loginfo([mapx_min, mapx_max])
        box = []
        bmin = Pose()
        minxy = self.pixel2Map(np.array([mapx_min, mapy_min]))
        bmin.position.x = minxy[0]
        bmin.position.y = minxy[1]
        box.append(bmin)
        bmax = Pose()
        maxxy = self.pixel2Map(np.array([mapx_max, mapy_max]))
        bmax.position.x = maxxy[0]
        bmax.position.y = maxxy[1]
        box.append(bmax)
        box_array = PoseArray()
        box_array.header.frame_id = 'map'
        box_array.poses = box
        self.box_pub.publish(box_array)

        rectangle = self.data[mapy_min:mapy_max, mapx_min:mapx_max]
        max_rect = np.amax(rectangle)
        # rospy.loginfo(max_rect)
        collision=False
        if max_rect>0:
            collision=True
        return collision

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
