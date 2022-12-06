#!/usr/bin/env python
from __future__ import print_function

from numpy import linalg as LA
from multiprocessing import Process
import threading
from tqdm import tqdm
from tf.transformations import *
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi, tau, fabs, cos, tan, atan
from std_msgs.msg import ColorRGBA, Header
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import copy
from six.moves import input
import argparse
import random
import sys

import rospkg

sys.path.insert(1, rospkg.RosPack().get_path('wtr_navigation') + "/src/strategizer/")

import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import numpy as np
import rospy
from wtr_navigation.srv import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


from strategizer import strategizer
from tennis import Tennis
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

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print(planning_frame)
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        display_waypoints_pub = rospy.Publisher(
            "/swing_waypoints", Marker, queue_size=20, latch=True)
        display_success_points_pub = rospy.Publisher(
            "/success_points", Marker, queue_size=20, latch=True)

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

def observation_callback(path_msg):
    ball_p = None
    ball_pose = None
    for pose in path_msg.poses:
        pose = pose.pose
        if abs(pose.position.y) < .01:
            ball_p = pose.position
            ball_pose = pose
            break

    if ball_p is None:
        return
    # print(ball_pose)

    # ball_p = Point(1.339, -0.028, 1.093)
    # ori = Quaternion(0.009, -0.009, -0.707, 0.707)


    ball_ps = PointStamped(header=Header(frame_id="world"), point=ball_p)
    # ball_ps.point.x -= .3
    if not (0.25 < ball_ps.point.x < 2):
      return

    if not (.7 < ball_ps.point.z < 2.5):
      return

    global relative_ball_ps
    relative_ball_ps = ball_ps


def go_ball(event):
  if relative_ball_ps:
    hit_pub.publish(relative_ball_ps)

    try:
      response = swing_service(
          ball_position=relative_ball_ps, swing_direction=Vector3(0, .1, 0))
      # print('[test hit] percent complete', response.percent_complete)
    except rospy.ServiceException as e:
       pass
        # print('[test hit] Swing call failed!')




def main():
    rospy.init_node("test_ball_traj", anonymous=True)

    global interface
    # interface = MoveGroupInterface()
    # interface.move_group.set_max_acceleration_scaling_factor(0.1)
    # interface.move_group.set_max_velocity_scaling_factor(0.1)

    global pub_cmd, client, ball_trajectory_pub, hit_pub, marker_pub, pub_traj_reset, pub_localize_reset
    global swing_service

    # Get Ball Position
    rospy.Subscriber("/ball/rollout", Path, observation_callback, queue_size=1)

    # Hit Ball Position
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    global listener
    listener = tf.TransformListener()

    print("[Planner] Waiting for LinearSwing server...")
    rospy.wait_for_service('/linear_stroke_move')
    print("[Planner] Connected to LinearSwing server...")
    swing_service = rospy.ServiceProxy('/linear_stroke_move', LinearSwing)

    publish_state_timer = rospy.Timer(rospy.Duration(0.5), go_ball)

    rospy.spin()

if __name__ == '__main__':
    main()
