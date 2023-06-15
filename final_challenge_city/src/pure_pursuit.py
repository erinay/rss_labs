#!/usr/bin/env python
import rospy
import numpy as np
import time
import utils
import tf
import tf2_ros
from std_msgs.msg import Float32

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from final_challenge_city.msg import StopDriveCmd

# class CannotFindGoal(Exception):
#     '''Could not find a goal'''
#     pass
#
# class PurePursuit(object):
#     """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
#     """
#     def __init__(self):
#
#         self.odom_topic = rospy.get_param("~odom_topic")
#         self.drive_topic = rospy.get_param("~drive_topic")
#         # self.lookahead = 0.75
#         # self.speed = .690
#         self.lookahead = 0.5
#         self.speed = .50
#         self.wheelbase_length = 0.8
#         self.stop_timer = 2.0
#
#         self.goal_thresh = .05 # how close we can be to our goal position
#
#         self.trajectory = utils.LineTrajectory("/followed_trajectory")
#
#
#         self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
#
#         # self.stopping_controller = rospy.Subscriber("/stopping_controller", StopDriveCmd, self.stopping_callback, queue_size=1)
#
#         self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
#
#         self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
#
#         self.goal_pub = rospy.Publisher("/goal_point", Marker, queue_size=1)
#
#         self.stop_detected = 0
#         self.stop_speed = 0
#         self.stop_steering_angle = False
#
#         self.in_stop_loop = False
#         self.start_loop_time = 0
#
#         self.on_final_seg = False
#         self.final_goal_x = None
#         self.final_goal_y = None
#
#
#     def trajectory_callback(self, msg):
#         ''' Clears the currently followed trajectory, and loads the new one from the message
#         '''
#         print("Receiving new trajectory:", len(msg.poses), "points")
#         self.trajectory.clear()
#         self.trajectory.fromPoseArray(msg)
#         self.trajectory.publish_viz(duration=0.0)
#
#     # def stopping_callback(self, stop_drive_msg):
#     #     if self.stop_detected == 0 and stop_drive_msg.sign_detection == 1: # check to see if we just started stopping or not
#     #         self.in_stop_loop = True
#     #         now = rospy.get_rostime()
#     #         self.start_loop_time = now.to_sec()
#     #
#     #
#     #     self.stop_detected = stop_drive_msg.sign_detection
#     #     self.stop_speed = stop_drive_msg.speed
#     #     self.stop_steering_angle = stop_drive_msg.steering_angle
#
#     def odom_callback(self, odom):
#
#         if len(self.trajectory.points) > 1:
#
#             self.car_pose = odom.pose.pose
#             (min_dist_seg, dist_arr) = self.get_segment_closest_to_robot()
#             # (goal_x, goal_y) = (None, None)
#             (goal_x, goal_y) = self.find_goal_point(min_dist_seg)
#             for seg in range(1,4):
#                 segment = seg + min_dist_seg
#                 temp_goal = self.find_goal_point(segment)
#                 if segment == (len(self.trajectory.points)-2) and temp_goal != (None, None): # check for final segment goal point
#                     (self.final_goal_x, self.final_goal_y) = self.trajectory.points[-1]
#                     (goal_x, goal_y) = (self.final_goal_x, self.final_goal_y)
#                     # ^^ sets the goal point as the final end pose
#                     self.on_final_seg = True
#                     rospy.loginfo("on final seg")
#                     # enters a final segment loop so we always drive towards correct location
#                 if temp_goal != (None, None):
#                     (goal_x, goal_y) = temp_goal
#             if self.on_final_seg or (goal_x != None):
#
#                 self.add_marker(goal_x, goal_y)
#                 drive_angle = self.find_drive_angle(goal_x, goal_y)
#                 # self.publish_error(dist_arr, min_dist_seg)
#                 self.drive_command(drive_angle, goal_x, goal_y)
#
#     def get_segment_closest_to_robot(self):
#
#         trajectory_points = np.array(self.trajectory.points)
#         robot_x = self.car_pose.position.x
#         robot_y = self.car_pose.position.y
#         x1_arr = trajectory_points[:-1, 0]
#         x2_arr = trajectory_points[1:, 0]
#         y1_arr = trajectory_points[:-1, 1]
#         y2_arr = trajectory_points[1:, 1]
#         px = x2_arr - x1_arr
#         py = y2_arr - y1_arr
#         norm = np.square(px) + np.square(py)
#         u = ((robot_x - x1_arr) * px + (robot_y - y1_arr) * py) / norm
#         u_clip = np.clip(u, 0, 1)
#         x = x1_arr + u_clip * px
#         y = y1_arr + u_clip * py
#         dx = x - robot_x
#         dy = y - robot_y
#         dist_arr = np.sqrt(np.square(dx) + np.square(dy))
#         min_dist_segment = np.argmin(dist_arr)
#         # rospy.loginfo("looking at segment: %s", min_dist_segment)
#         return (min_dist_segment, dist_arr)
#
#     def find_goal_point(self,segment_idx):
#         if segment_idx >= len(self.trajectory.points)-1:
#             return (None, None)
#         begin = self.trajectory.points[segment_idx]
#         end = self.trajectory.points[segment_idx+1]
#         Q = np.array([self.car_pose.position.x, self.car_pose.position.y])  # Center of Robot
#         r = self.lookahead  # radius
#         P1 = np.array([begin[0], begin[1]])  # position of beginning of line segment
#         P2 = np.array([end[0], end[1]])  # position of end of line segment
#         V = P2 - P1  # vector that moves along the line segment
#
#         ## There exists a t such that P1 + V*t = intersection. These are coeffs for t
#         a = V.dot(V)
#         b = 2 * V.dot(P1 - Q)
#         c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r ** 2
#
#         # Line misses the circle
#         disc = b ** 2 - 4 * a * c
#         if disc < 0:
#             return (None, None)
#
#         # calculates the 2 different intersection points with the lookahead circle
#         sqrt_disc = np.sqrt(disc)
#         t1 = (-b + sqrt_disc) / (2 * a)
#         t2 = (-b - sqrt_disc) / (2 * a)
#
#         # handles case where we miss, but extending vector would work
#         if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
#             return (None, None)
#
#         goal_temp_1 = P1 + t1 * V
#         goal_temp_2 = P1 + t2 * V
#
#         goal_position = self.better_path(goal_temp_1, goal_temp_2, segment_idx)
#
#         goal_x = goal_position[0]
#         goal_y = goal_position[1]
#         return (goal_x, goal_y)
#
#     def better_path(self, goal_temp_1, goal_temp_2, segment_idx):
#         P2 = self.trajectory.points[segment_idx+1]
#         dist_1 = np.sqrt((goal_temp_1[0] - P2[0]) ** 2 + (goal_temp_1[1] - P2[1]) ** 2)
#         dist_2 = np.sqrt((goal_temp_2[0] - P2[0]) ** 2 + (goal_temp_2[1] - P2[1]) ** 2)
#         if dist_1 < dist_2:
#             return goal_temp_1
#         return goal_temp_2
#
#     def find_drive_angle(self, goal_x, goal_y):
#         if self.on_final_seg:
#             goal_x = self.final_goal_x
#             goal_y = self.final_goal_y
#         rospy.loginfo("goal_x is %s", goal_x)
#         rospy.loginfo("goal_y is %s", goal_y)
#         tr_world2goal = np.array([[1, 0, goal_x],[0, 1, goal_y],[0,0,1]])
#         car_trans = self.car_pose.position
#         car_quat = self.car_pose.orientation
#         car_rot = tf.transformations.quaternion_matrix([car_quat.x, car_quat.y, car_quat.z, car_quat.w])
#         tr_world2car = np.array([[car_rot[0, 0], car_rot[0, 1], car_trans.x],[car_rot[1, 0], car_rot[1, 1], car_trans.y],[0,0,1]])
#         trcar2world = np.linalg.inv(tr_world2car)
#         tr_car2goal = np.matmul(trcar2world,tr_world2goal)
#
#         rel_x = tr_car2goal[0][2]
#         rel_y = tr_car2goal[1][2]
#
#         drive_angle = np.arctan2(rel_y, rel_x)
#
#         return drive_angle
#
#     def add_marker(self, goal_x, goal_y):
#         marker = Marker()
#         marker.type = marker.CYLINDER
#         marker.action = marker.ADD
#         marker.scale.x = 0.2
#         marker.scale.y = 0.2
#         marker.scale.z = 0.2
#         marker.color.a = 1.0
#         marker.color.r = 1.0
#         marker.color.g = 0.5
#
#         marker.header.frame_id = "map"
#         marker.pose.position.x = goal_x
#         marker.pose.position.y = goal_y
#         marker.pose.position.z = 0
#         marker.pose.orientation.x = 0
#         marker.pose.orientation.y = 0
#         marker.pose.orientation.z = 0
#         marker.pose.orientation.w = 1
#
#         self.goal_pub.publish(marker)
#
#     # def publish_error(self, dist_arr, traj_idx):
#     #     dist_from_robot = dist_arr[traj_idx]
#     #     dist_from_robot = dist_from_robot.astype(Float32)
#     #     self.error_pos_pub.publish(dist_from_robot)
#
#     def drive_command(self, drive_angle, goal_x, goal_y):
#
#         if self.car_pose is None:
#             return
#         drive_stamp = AckermannDriveStamped()
#         final_pos_x = self.trajectory.points[-1][0]
#         final_pos_y = self.trajectory.points[-1][1]
#
#         # update_time = rospy.get_rostime()
#         # curr_loop_time = update_time.to_sec()
#         # if self.in_stop_loop:
#         #     if curr_loop_time - self.start_loop_time >= self.stop_timer:
#         #         self.in_stop_loop = False
#         #
#         # if self.stop_detected == 1 and self.in_stop_loop:
#         #     rospy.loginfo("drive_cmd stop sign")
#         #     drive_stamp.drive.steering_angle = self.stop_steering_angle
#         #     drive_stamp.drive.speed = self.stop_speed
#
#         robot_x = self.car_pose.position.x
#         robot_y = self.car_pose.position.y
#         dist_from_goal = np.sqrt(np.square(final_pos_x - robot_x) + np.square(final_pos_y - robot_y))
#
#         if dist_from_goal < self.goal_thresh:
#             drive_stamp.drive.steering_angle = 0
#             drive_stamp.drive.speed = 0
#             self.on_final_seg = False
#
#         else:
#             drive_stamp.drive.steering_angle = drive_angle
#             drive_stamp.drive.speed = self.speed
#
#         self.drive_pub.publish(drive_stamp)
#
# if __name__=="__main__":
#     rospy.init_node("pure_pursuit")
#     pf = PurePursuit()
#     rospy.spin()

############################################

class CannotFindGoal(Exception):
    '''Could not find a goal'''
    pass

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):

        self.odom_topic = rospy.get_param("~odom_topic")
        self.drive_topic = rospy.get_param("~drive_topic")
        self.lookahead = 0.75
        self.speed = .69
        # self.lookahead = 0.5
        # self.speed = .5
        self.wheelbase_length = 0.8

        self.stop_timer = 2.0

        self.backwards_time = 4.0
        self.backwards_speed = -0.7

        self.goal_thresh = .04 # how close we can be to our goal position

        self.trajectory = utils.LineTrajectory("/followed_trajectory")


        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)

        # self.stopping_controller = rospy.Subscriber("/stopping_controller", StopDriveCmd, self.stopping_callback, queue_size=1)

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)

        self.goal_pub = rospy.Publisher("/goal_point", Marker, queue_size=1)

        self.stop_detected = 0
        self.stop_speed = 0
        self.stop_steering_angle = False

        self.in_stop_loop = False
        self.start_loop_time = 0

        self.on_final_seg = False
        self.final_goal_x = None
        self.final_goal_y = None

        self.in_first_traj = True
        self.reached_goal_one_time = -10
        self.first_end_idx = 14
        self.second_start_idx = 15
        self.in_backwards_loop = False

        self.completed_final_journey = False


    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.traj_first_points = self.trajectory.points[:self.first_end_idx]
        self.traj_second_points = self.trajectory.points[self.second_start_idx:]


    # def stopping_callback(self, stop_drive_msg):
    #     if self.stop_detected == 0 and stop_drive_msg.sign_detection == 1: # check to see if we just started stopping or not
    #         self.in_stop_loop = True
    #         now = rospy.get_rostime()
    #         self.start_loop_time = now.to_sec()
    #
    #
    #     self.stop_detected = stop_drive_msg.sign_detection
    #     self.stop_speed = stop_drive_msg.speed
    #     self.stop_steering_angle = stop_drive_msg.steering_angle

    def odom_callback(self, odom):

        if len(self.trajectory.points) > 1:
            if self.in_first_traj:
                trajectory_points = self.traj_first_points
            else:
                rospy.loginfo("SWAPPED TO SECOND TRAJ")
                trajectory_points = self.traj_second_points
            self.car_pose = odom.pose.pose
            (min_dist_seg, dist_arr) = self.get_segment_closest_to_robot(trajectory_points)
            (goal_x, goal_y) = self.find_goal_point(min_dist_seg, trajectory_points)
            for seg in range(1,4):
                segment = seg + min_dist_seg
                temp_goal = self.find_goal_point(segment, trajectory_points)
                if segment == (len(trajectory_points) - 2) and temp_goal != (None, None): # check for final segment goal point
                    (self.final_goal_x, self.final_goal_y) = trajectory_points[-1]
                    (goal_x, goal_y) = (self.final_goal_x, self.final_goal_y)
                    # ^^ sets the goal point as the final end pose
                    self.on_final_seg = True
                    # enters a final segment loop so we always drive towards correct location
                if temp_goal != (None, None):
                    (goal_x, goal_y) = temp_goal
            if goal_x or self.on_final_seg or self.in_backwards_loop:
                if goal_x:
                    self.add_marker(goal_x, goal_y)
                drive_angle = self.find_drive_angle(goal_x, goal_y)
                # self.publish_error(dist_arr, min_dist_seg)
                self.drive_command(drive_angle, goal_x, goal_y, trajectory_points)

    def get_segment_closest_to_robot(self, trajectory_points):

        trajectory_points = np.array(trajectory_points)
        robot_x = self.car_pose.position.x
        robot_y = self.car_pose.position.y
        x1_arr = trajectory_points[:-1, 0]
        x2_arr = trajectory_points[1:, 0]
        y1_arr = trajectory_points[:-1, 1]
        y2_arr = trajectory_points[1:, 1]
        px = x2_arr - x1_arr
        py = y2_arr - y1_arr
        norm = np.square(px) + np.square(py)
        u = ((robot_x - x1_arr) * px + (robot_y - y1_arr) * py) / norm
        u_clip = np.clip(u, 0, 1)
        x = x1_arr + u_clip * px
        y = y1_arr + u_clip * py
        dx = x - robot_x
        dy = y - robot_y
        dist_arr = np.sqrt(np.square(dx) + np.square(dy))
        min_dist_segment = np.argmin(dist_arr)
        # rospy.loginfo("looking at segment: %s", min_dist_segment)
        return (min_dist_segment, dist_arr)

    def find_goal_point(self,segment_idx, trajectory_points):
        if segment_idx >= len(trajectory_points)-1:
            return (None, None)
        begin = trajectory_points[segment_idx]
        end = trajectory_points[segment_idx+1]
        Q = np.array([self.car_pose.position.x, self.car_pose.position.y])  # Center of Robot
        r = self.lookahead  # radius
        P1 = np.array([begin[0], begin[1]])  # position of beginning of line segment
        P2 = np.array([end[0], end[1]])  # position of end of line segment
        V = P2 - P1  # vector that moves along the line segment

        ## There exists a t such that P1 + V*t = intersection. These are coeffs for t
        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r ** 2

        # Line misses the circle
        disc = b ** 2 - 4 * a * c
        if disc < 0:
            return (None, None)

        # calculates the 2 different intersection points with the lookahead circle
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        # handles case where we miss, but extending vector would work
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return (None, None)

        goal_temp_1 = P1 + t1 * V
        goal_temp_2 = P1 + t2 * V

        goal_position = self.better_path(goal_temp_1, goal_temp_2, segment_idx, trajectory_points)

        goal_x = goal_position[0]
        goal_y = goal_position[1]
        return (goal_x, goal_y)

    def better_path(self, goal_temp_1, goal_temp_2, segment_idx, trajectory_points):
        P2 = trajectory_points[segment_idx+1]
        dist_1 = np.sqrt((goal_temp_1[0] - P2[0]) ** 2 + (goal_temp_1[1] - P2[1]) ** 2)
        dist_2 = np.sqrt((goal_temp_2[0] - P2[0]) ** 2 + (goal_temp_2[1] - P2[1]) ** 2)
        if dist_1 < dist_2:
            return goal_temp_1
        return goal_temp_2

    def find_drive_angle(self, goal_x, goal_y):
        if not self.in_backwards_loop:
            if self.on_final_seg:
                goal_x = self.final_goal_x
                goal_y = self.final_goal_y
            tr_world2goal = np.array([[1, 0, goal_x],[0, 1, goal_y],[0,0,1]])
            car_trans = self.car_pose.position
            car_quat = self.car_pose.orientation
            car_rot = tf.transformations.quaternion_matrix([car_quat.x, car_quat.y, car_quat.z, car_quat.w])
            tr_world2car = np.array([[car_rot[0, 0], car_rot[0, 1], car_trans.x],[car_rot[1, 0], car_rot[1, 1], car_trans.y],[0,0,1]])
            trcar2world = np.linalg.inv(tr_world2car)
            tr_car2goal = np.matmul(trcar2world,tr_world2goal)

            rel_x = tr_car2goal[0][2]
            rel_y = tr_car2goal[1][2]

            drive_angle = np.arctan2(rel_y, rel_x)

            return drive_angle

    def add_marker(self, goal_x, goal_y):
        marker = Marker()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5

        marker.header.frame_id = "map"
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        self.goal_pub.publish(marker)

    # def publish_error(self, dist_arr, traj_idx):
    #     dist_from_robot = dist_arr[traj_idx]
    #     dist_from_robot = dist_from_robot.astype(Float32)
    #     self.error_pos_pub.publish(dist_from_robot)

    def drive_command(self, drive_angle, goal_x, goal_y, trajectory_points):

        if self.car_pose is None:
            return
        drive_stamp = AckermannDriveStamped()
        final_pos_x = trajectory_points[-1][0]
        final_pos_y = trajectory_points[-1][1]

        robot_x = self.car_pose.position.x
        robot_y = self.car_pose.position.y
        dist_from_goal = np.sqrt(np.square(final_pos_x - robot_x) + np.square(final_pos_y - robot_y))

        update_time = rospy.get_rostime()
        curr_time = update_time.to_sec()
        # if self.in_stop_loop:
        #     if curr_time - self.start_loop_time >= self.stop_timer:
        #         self.in_stop_loop = False
        #
        # if self.stop_detected == 1 and self.in_stop_loop:
        #     rospy.loginfo("drive_cmd stop sign")
        #     drive_stamp.drive.steering_angle = self.stop_steering_angle
        #     drive_stamp.drive.speed = self.stop_speed

        if self.in_first_traj == False and dist_from_goal < self.goal_thresh:
            self.completed_final_journey = True
            drive_stamp.drive.steering_angle = 0
            drive_stamp.drive.speed = 0

        elif self.in_backwards_loop:
            if curr_time - self.reached_goal_one_time > self.backwards_time:
                self.in_backwards_loop = False
            drive_stamp.drive.steering_angle = 0
            drive_stamp.drive.speed = self.backwards_speed


        elif dist_from_goal < self.goal_thresh:
            rospy.loginfo("REACHED FIRST GOAL")
            drive_stamp.drive.steering_angle = 0
            drive_stamp.drive.speed = 0
            self.on_final_seg = False
            self.in_first_traj = False
            self.in_backwards_loop = True
            portal_one_time = rospy.get_rostime()
            self.reached_goal_one_time = portal_one_time.to_sec()
            rospy.sleep(1) # stop the robot for 1 second to park

        else:
            drive_stamp.drive.steering_angle = drive_angle
            drive_stamp.drive.speed = self.speed

        self.drive_pub.publish(drive_stamp)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
