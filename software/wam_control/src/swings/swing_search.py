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
        move_group.set_planner_id("PersistentLazyPRMstar")

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        display_waypoints_pub = rospy.Publisher("/swing_waypoints", Marker, queue_size=20)
        display_success_points_pub = rospy.Publisher("/success_points", Marker, queue_size=20)

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.display_waypoints_pub = display_waypoints_pub
        self.display_success_points_pub = display_success_points_pub


    def make_swing_path(self, point):
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


    def plan_cartesian_path(self, waypoints, scale=1):
      (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.1, 0.0)
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


    def execute_plan(self, plan):
      self.move_group.execute(plan, wait=True)


box = [[-1, 1], [-1, 1], [0, 2]]
exclude_box =[[]]

def main():

  ball_points = []
  interface = MoveGroupInterface()

  time.sleep(1)

  (min_x, max_x), (min_y, max_y), (min_z, max_z) = ((-1.5, 1.5), (-1.5, 1.5), (0, 2))

  # Exclude inner box
  (ex_min_x, ex_max_x), (ex_min_y, ex_max_y) = ((-0.6, 0.6), (-0.6, 0.6))

  step = 0.2

  for x in tqdm(np.arange(min_x, max_x, step), desc='x-loop'):
    for y in tqdm(np.arange(min_y, max_y, step), desc='y-loop', leave=False):
      for z in tqdm(np.arange(min_z, max_z, step), desc='z-loop', leave=False):
        if (ex_min_x < x and x < ex_max_x) and (ex_min_y < y and y < ex_max_y):
          continue

        point = Point(x, y, z)
        waypoints = interface.make_swing_path(point)
        cartesian_plan, fraction = interface.plan_cartesian_path(waypoints)
        success = (fraction > .9)
        ball_points.append([point, success])

  interface.display_points(ball_points)

  # Get offset that has most successful heights 
  offsets = dict()
  for ball_point in ball_points:
    point, success = ball_point
    key = str(point.x) + "," + str(point.y)
    if success:
      if key not in offsets:
        offsets[key] = 1
      else:
        offsets[key] += 1

  for k, v in sorted(offsets.items(), key=lambda kv: kv[1], reverse=True):
    print("%s => %s" % (k, v))



  # interface.display_waypoints(waypoints)
  # interface.display_trajectory(cartesian_plan)


if __name__ == "__main__":
  main()
