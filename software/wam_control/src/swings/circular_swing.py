from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
from math import pi, tau, fabs, cos, tan, atan

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import *
import time

import numpy as np

from tqdm import tqdm

import threading
from multiprocessing import Process

from numpy import linalg as LA

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
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("wam/racquet_hitpoint_link")
        move_group.set_planning_time(5)
        # move_group.set_planner_id("RRTstarkConfigDefault")

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        display_waypoints_pub = rospy.Publisher("/swing_waypoints", Marker, queue_size=20, latch=True)
        display_success_points_pub = rospy.Publisher("/success_points", Marker, queue_size=20, latch=True)


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
      plan = self.move_group.retime_trajectory(self.robot.get_current_state(), plan, 1, 1)
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

def make_circular_swing(point):
  px, py, pz = point.x, point.y, point.z
  r = LA.norm([px, py], 2)

  start_angle = np.pi / 2
  end_angle = 0

  waypoints = []
  for theta in np.linspace(start_angle, end_angle, 10):
    # print(theta)
    pose = Pose()
    pose.position = Point(r * np.cos(theta), r * np.sin(theta), pz)
    q_tf = quaternion_from_euler(np.pi/2-theta, np.pi/2, 0)
    pose.orientation = Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])
    waypoints.append(pose)

  return waypoints


def main():
  # time.sleep(2)
  interface = MoveGroupInterface()
  interface.move_group.set_max_acceleration_scaling_factor(0.1)
  interface.move_group.set_max_velocity_scaling_factor(0.1)

  point = Point(0, 1, 1)
  waypoints = make_circular_swing(point)
  interface.move_pose(waypoints[0])
  print("arrived")
  # interface.move_poses(waypoints)
  cartesian_plan, fraction = interface.plan_cartesian_path(waypoints[1:])
  success = (fraction == 1)
  print(success)



  # for _ in range(10):
  interface.display_waypoints(waypoints)
  interface.display_trajectory(cartesian_plan)

  interface.execute_plan(cartesian_plan)
  print(cartesian_plan)

  rospy.spin()


if __name__ == "__main__":
  main()
