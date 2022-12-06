#!/usr/bin/env python
from __future__ import print_function
from visualization_msgs.msg import Marker
from trac_ik_python.trac_ik import IK
from turtle import position

from numpy import linalg as LA
from multiprocessing import Process
import threading
from tf.transformations import *
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi, tau, fabs, cos, tan, atan
from std_msgs.msg import ColorRGBA, Header
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import copy
import numpy as np
from six.moves import input
import argparse
import random
import sys

import rospkg

sys.path.insert(1, rospkg.RosPack().get_path('wtr_navigation') + "/src/strategizer/")

import actionlib
import actionlib_msgs.msg
import numpy as np
import rospy
from wtr_navigation.srv import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler

from wam_control.srv import *

# from strategizer import strategizer
# from tennis import Tennis
import time
from nav_msgs.msg import Path

import numpy as np

import tf

pub_cmd = None
marker_pub = None
pub_traj_reset = None
pub_localize_reset = None
strat = None
client = None
hit_pos = None
swing_type = None
listener = None

bounce_state = 0    # 0 = pre-bounce, 1 = post-bounce
last_velz = None

ball_odometry = None
robot_odometry = None
ball_trajectory_pub = None
hit_pub = None

swing_service = None

SIM = False
ik_solver = None


max_velocity_scaling_factor = 0.1
max_acceleration_scaling_factor = 0.1


class MoveGroupInterface(object):
    """MoveGroupInterface"""

    def __init__(self):
        super(MoveGroupInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(
            "arm", wait_for_servers=5)
        move_group.set_end_effector_link("wam/racquet_hitpoint_link")
        move_group.set_planning_time(5)
        # move_group.set_planner_id("RRTstarkConfigDefault")

        move_group.set_max_acceleration_scaling_factor(1)
        move_group.set_max_velocity_scaling_factor(1)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        print(f"Planning frame: {planning_frame}, eef link: {eef_link}")

        group_names = robot.get_group_names()

        display_waypoints_pub = rospy.Publisher(
            "/swing_waypoints", Marker, queue_size=1, latch=True)
        display_success_points_pub = rospy.Publisher(
            "/success_points", Marker, queue_size=1, latch=True)

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.display_trajectory_publisher = display_trajectory_publisher
        self.display_waypoints_pub = display_waypoints_pub
        self.display_success_points_pub = display_success_points_pub

        self.traj_generator = rospy.ServiceProxy('/trajectory_profiler', TrajectoryProfiler)
        self.prev_hit_point = Point(0,0,0)
        self.prev_time_s = 0

    def plan_cartesian_path(self, waypoints, scale=1):
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 100, 0.0)
      plan = self.move_group.retime_trajectory(
          self.robot.get_current_state(), plan, 1, 1)
      return plan, fraction

    def display_trajectory(self, plan):
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = self.robot.get_current_state()
      display_trajectory.trajectory.append(plan)
      self.display_trajectory_publisher.publish(display_trajectory)

    def display_waypoints(self, waypoints):
      marker = Marker(header=Header(frame_id=self.planning_frame),
                      type=Marker.LINE_STRIP,
                      pose=Pose(orientation=Quaternion(w=1)),
                      points=[p.position for p in waypoints],
                      scale=Vector3(.01, .01, .01),
                      color=ColorRGBA(0, 1, 0, 1))
      self.display_waypoints_pub.publish(marker)

    def display_points(self, points):
      marker = Marker(header=Header(frame_id=self.planning_frame),
                      type=Marker.POINTS,
                      pose=Pose(orientation=Quaternion(w=1)),
                      points=[p[0] for p in points],
                      scale=Vector3(.01, .01, .01),
                      colors=[ColorRGBA(1*(not p[1]), 1*p[1], 0, 1) for p in points])
      self.display_success_points_pub.publish(marker)

    def move_pose(self, pose):
      self.move_group.set_pose_target(pose)
      self.move_group.go(wait=True)
      # self.move_group.stop()
      self.move_group.clear_pose_targets()

    def move_poses(self, poses):
      self.move_group.set_pose_targets(poses)
      plan = self.move_group.go(wait=True)
      # self.move_group.stop()
      self.move_group.clear_pose_targets()

    def execute_plan(self, plan):
      self.move_group.execute(plan, wait=True)

relative_ball_ps = None
confidence = False

def path_callback(path_msg):
    s = time.time()
    ball_position = None
    # Find where rollout intercepts y-plane
    for pose_stamped in path_msg.poses:
        pose = pose_stamped.pose
        if abs(pose.position.y) < .02 and pose_stamped.header.seq and 0.5 < pose.position.z < 2 and 0.5 < pose.position.x < 2:
            ball_position = pose.position
            break

    # Ball intercept not found
    if ball_position is None:
        return

    ball_ps = PointStamped(header=Header(
        frame_id="world"), point=ball_position)
    hit_pub.publish(ball_ps)

    # print("hitpoint:", ball_ps)
    rel_ball_ps = listener.transformPoint("wam/base_link", ball_ps)

    # calculate distance from last hitpoint to determine if replanning is needed
    prev_x = interface.prev_hit_point.x
    prev_y = interface.prev_hit_point.y
    prev_z = interface.prev_hit_point.z
    prev_pt = np.array([prev_x, prev_y, prev_z])

    cur_x = ball_position.x
    cur_y = ball_position.y
    cur_z = ball_position.z
    cur_pt = np.array([cur_x, cur_y, cur_z])


    hp_dist = np.linalg.norm(prev_pt-cur_pt)
    # print("dist", hp_dist)
    # if interface.prev_time_s == 0:
    #     interface.prev_time_s = rospy.get_time()
    
    time_s = rospy.get_time()
    elapses_time_s = time_s - interface.prev_time_s

    if hp_dist > 0.2: #and elapses_time_s > 0.2:
        interface.prev_time_s = time_s
        # print(elapses_time_s)
        # print("Replanning... dist:", hp_dist)
        

        interface.prev_hit_point = ball_position
        curr_joints = interface.move_group.get_current_joint_values()
        seed_state = list(curr_joints)
        ik_joints = ik_solver.get_ik(seed_state,
                                rel_ball_ps.point.x, rel_ball_ps.point.y, rel_ball_ps.point.z,
                                -0.046, 0.713, -0.698, -0.050,
                                0.05, 0.05, 0.05,  # X, Y, Z bounds
                                1, 1, 100)      # Rotation X, Y, Z bounds

        if ik_joints:
            for i in range(len(curr_joints)):
                curr_joints[i] = ik_joints[i]
            # print("IK Joints", ik_joints)
            # print("Elapsed time:", elapses_time_s)
            # print("IK Solution Found")
            # interface.move_group.set_joint_value_target(curr_joints)
            # interface.move_group.go(wait=False)

            service_request = TrajectoryProfilerRequest()
            service_request.goal_positions = list(ik_joints)
            # service_request.desired_joints = [0.0, math.pi/2, 0.0, -math.pi]
            try:
                resp = interface.traj_generator(service_request)
                print("resp:", resp)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)

            
        else:
            # print("IK Failed")
            pass

        print(time.time() -s, "sec: all", 1 if ik_joints else 0)


def spawn_random_ball(event):
    print("Spawn ball")
    vy = np.random.normal(-8, .1)
    vx = np.random.normal(0, .1)
    vz = np.random.normal(-4, .3)
    # Spawn Random Ball
    pose = Pose(position=Point(1, 5, 1), orientation=Quaternion(w=1))
    pub_cmd.publish(Odometry(header=Header(frame_id="world"),
                             pose=PoseWithCovariance(pose=pose),
                             twist=TwistWithCovariance(
                                 twist=Twist(linear=Vector3(vx, vy, vz)))
                             ))
    time.sleep(.1)
    pub_traj_reset.publish(String("reset"))
    pub_localize_reset.publish(PoseWithCovarianceStamped(
        header=Header(frame_id="world",
                      stamp=rospy.Time.now()),
        pose=PoseWithCovariance(pose=pose)))


def main():
    # rospy.init_node("test_ball_traj", anonymous=True)

    global interface
    interface = MoveGroupInterface()

    global pub_cmd, client, ball_trajectory_pub, hit_pub, marker_pub, pub_traj_reset, pub_localize_reset
    global swing_service

    # Get Ball Position
    rospy.Subscriber("/ball/rollout/path", Path, path_callback, queue_size=1)

    # Hit Ball Position
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # Command Ball Position
    pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

    # Reset
    pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
    pub_localize_reset = rospy.Publisher(
        "/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

    global listener
    listener = tf.TransformListener()

    # Spawn ball on timer
    # spawn_ball_timer = rospy.Timer(rospy.Duration(5), spawn_random_ball)

    urdf_str = rospy.get_param('/robot_description')

    global ik_solver
    ik_solver = IK("wam/base_link", "wam/racquet_hitpoint_link",
                   urdf_string=urdf_str)

    rospy.spin()

if __name__ == '__main__':
    main()
