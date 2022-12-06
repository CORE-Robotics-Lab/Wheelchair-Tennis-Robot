#!/usr/bin/env python
from __future__ import print_function
from turtle import position

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

sys.path.insert(1, rospkg.RosPack().get_path('wtr_plan') + "/src/strategizer/")

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

SIM = True


# max_velocity_scaling_factor = 0.1
# max_acceleration_scaling_factor = 0.1


class MoveGroupInterface(object):
    """MoveGroupInterface"""

    def __init__(self):
        super(MoveGroupInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(
            "arm", wait_for_servers=5)
        move_group.set_end_effector_link("wam/racquet_hitpoint_link")
        move_group.set_planning_time(.2)
        # move_group.set_planner_id("RRTstarkConfigDefault")

        move_group.set_max_acceleration_scaling_factor(1)
        move_group.set_max_velocity_scaling_factor(1)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        print(f"Planning frame: {planning_frame}, eef link: {eef_link}")

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
      self.move_group.set_goal_position_tolerance(0.1)
      self.move_group.set_goal_joint_tolerance(0.5)
      self.move_group.set_goal_orientation_tolerance(0.1)
      self.move_group.go(wait=False)
      # self.move_group.stop()
      self.move_group.clear_pose_targets()

    def move_poses(self, poses):
      self.move_group.set_pose_targets(poses)
      plan = self.move_group.go(wait=True)
      # self.move_group.stop()
      self.move_group.clear_pose_targets()

    def execute_plan(self, plan):
      self.move_group.execute(plan, wait=True)
      

# relative_ball_ps = None
# confidence = False

def make_linear_swing(point):
  waypoints = []

  strike_pitch = pi / 4
  distance = 0.1

  slope = tan(strike_pitch)

  base_pose = Pose()
  base_pose.position = point

  q_tf = quaternion_from_euler(0, strike_pitch, 0)
  base_pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])
  waypoints.append(base_pose)

  pose1 = copy.deepcopy(base_pose)
  pose1.position.x -= distance
  pose1.position.z -= slope * distance
  waypoints.append(pose1)

  pose2 = copy.deepcopy(base_pose)
  pose2.position.x += distance
  pose2.position.z += slope * distance
  waypoints.append(pose2)

  return waypoints

def observation_callback(odometry_msg):
    global ball_odometry, strat
    ball_odometry = odometry_msg

    tennis = Tennis(ball_odometry, 0)
    tennis.display_trajectory()

    ball_p = None
    for point_data in tennis.trajectory:
        if abs(point_data[0] - 0) < .01:
            ball_p = Point(point_data[0], point_data[1], point_data[2])
            break

    if ball_p is None:
        return

    ball_ps = PointStamped(header=Header(frame_id="world"), point=ball_p)
    hit_pub.publish(ball_ps)

    relative_ball_ps = listener.transformPoint("wam/base_link", ball_ps) 
    global ball_reset_cnt
    ball_reset_cnt += 1
    # if ball_reset_cnt == 15:
    if ball_reset_cnt > 8:
      test_goal = geometry_msgs.msg.Pose()  
      test_goal.orientation.x = 0.706011
      test_goal.orientation.y = 0.045011
      test_goal.orientation.z = -0.706011
      test_goal.orientation.w = -0.045011
      test_goal.position.x = ball_ps.point.x
      test_goal.position.y = ball_ps.point.y
      test_goal.position.z = ball_ps.point.z


      # linear swing test

      if test_goal.position.z > 0.5 and test_goal.position.z < 2.0:
        interface.move_pose(test_goal)
        planned_pub.publish(ball_ps)
      else:
        ball_reset_cnt -= 1
      rospy.loginfo("predicted x: " + str(ball_ps.point.x))
      rospy.loginfo("predicted y: " + str(ball_ps.point.y))
      rospy.loginfo("predicted z: " + str(ball_ps.point.z))
    # rospy.loginfo("ball cnt:" + str(ball_reset_cnt))


def spawn_random_ball(event):
    print("Spawn ball")
    vx = np.random.normal(-10, .1)
    vy = np.random.normal(0, .1)
    vz = np.random.normal(-4, .3)
    # Spawn Random Ball
    pose = Pose(position=Point(15, -1., 3), orientation=Quaternion(w=1))
    pub_cmd.publish(Odometry(header=Header(frame_id="world"),
                             pose=PoseWithCovariance(pose=pose),
                             twist=TwistWithCovariance(
                                 twist=Twist(linear=Vector3(vx, vy, vz)))
                             ))
    global ball_reset_cnt
    ball_reset_cnt = 0

    time.sleep(.1)
    pub_traj_reset.publish(String("reset"))
    pub_localize_reset.publish(PoseWithCovarianceStamped(
        header=Header(frame_id="world",
                      stamp=rospy.Time.now()),
        pose=PoseWithCovariance(pose=pose)))


def main():
    rospy.init_node("test_ball_traj", anonymous=True)

    # global interface
    # interface = MoveGroupInterface()

    global pub_cmd, client, ball_trajectory_pub, hit_pub, marker_pub, pub_traj_reset, pub_localize_reset
    global swing_service
    global interface, planned_pub

    interface = MoveGroupInterface()


    # Get Ball Position
    rospy.Subscriber("/ball_odometry", Odometry, observation_callback, queue_size=1)

    # Hit Ball Position
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # Planned end effector position
    planned_pub = rospy.Publisher("/planned_eef_point", PointStamped, queue_size=1)

    global listener
    listener = tf.TransformListener()

    if SIM:
        # Command Ball Position
        pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

        # Reset
        pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
        pub_localize_reset = rospy.Publisher(
            "/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)
        
        # Spawn ball on timer
        spawn_ball_timer = rospy.Timer(rospy.Duration(8.0), spawn_random_ball)
    
    print("[Planner] Waiting for LinearSwing server...")
    rospy.wait_for_service('/stroke_swing_service')
    print("[Planner] Connected to LinearSwing server...")
    swing_service = rospy.ServiceProxy('/stroke_swing_service', Swing)

    # # Spawn ball on timer
    # spawn_ball_timer = rospy.Timer(rospy.Duration(5), spawn_random_ball)

    rospy.spin()

if __name__ == '__main__':
    main()
