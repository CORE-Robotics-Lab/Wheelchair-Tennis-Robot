#!/usr/bin/env python
# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import sys
# sys.path.append("/home/dani/catkin_ws/src/Wheelchair-Tennis-Robot/Section_3-Control_and_Path_Planning/ROS_Packages/barrett_tennis/src/swings/pilz_robot_programming1/")
from numpy import linalg as LA
from multiprocessing import Process
import threading
from tqdm import tqdm
import numpy as np
import time
from tf.transformations import *
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from math import pi, tau, fabs, cos, tan, atan
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import *
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import copy
import math
from exceptions import *
from commands import*
from move_control_request import *


from geometry_msgs.msg import Point
import rospy
from robot import *


__REQUIRED_API_VERSION__ = "1"


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


def start_program():
    print("Executing " + __file__)

    r = Robot(__REQUIRED_API_VERSION__)

    # ball_p = Point(1.339, -0.028, 1.093)
    # ori = Quaternion(0.009, -0.009, -0.707, 0.707)
    # point = Point(0, -1, 1)
    r.move(Ptp(goal=Pose(position=Point(-0.620, -0.150, 1.427),
                         orientation=Quaternion(0.500, -0.500, -0.500, 0.500))))
    # waypoints = make_circular_swing(point)
    # print(waypoints[0].position)
    # r.move(Ptp(goal=, vel_scale=0.1, acc_scale=0.1))
    exit(0)

    # Simple ptp movement
    # print(wa)
    blend_sequence = Sequence()

    px, py, pz = point.x, point.y, point.z
    rad = LA.norm([px, py], 2)

    r.move(Circ(goal=waypoints[-1], center=Point(0.0, 0.0, 1), vel_scale=.1, acc_scale=.1))





    # for w in waypoints:
    #   blend_radius = 0.000001
    #   if w == waypoints[-1]:
    #     blend_radius = 0
    #   # r.move(Ptp(goal=w, vel_scale=1, acc_scale=1))
    #   blend_sequence.append(
    #       Ptp(goal=w, vel_scale=.5, acc_scale=.5), blend_radius=blend_radius)
    #   # r.move(Ptp(goal=[0, .5, .2, 0, 0, 0, 0], vel_scale=0.4))
    # r.move(blend_sequence)

    # start_joint_values = r.get_current_joint_states()

    # # Relative ptp movement
    # r.move(Ptp(goal=Pose(position=Point(0, 0, -0.1)), relative=True))
    # r.move(Ptp(goal=[0.2, 0, 0, 0, 0, 0, 0], relative=True, acc_scale=0.2))

    # pose_after_relative = r.get_current_pose()

    # # Simple Lin movement
    # r.move(Lin(goal=Pose(position=Point(0.2, 0, 0.8)), vel_scale=0.1, acc_scale=0.1))

    # # Relative Lin movement
    # r.move(Lin(goal=Pose(position=Point(0, -0.2, 0), orientation=from_euler(0, 0, math.radians(15))), relative=True,
    #            vel_scale=0.1, acc_scale=0.1))
    # r.move(Lin(goal=Pose(position=Point(0, 0.2, 0)), relative=True,
    #            vel_scale=0.1, acc_scale=0.1))

    # # Circ movement
    # r.move(Circ(goal=Pose(position=Point(0.2, -0.2, 0.8)),
    #             center=Point(0.1, -0.1, 0.8), acc_scale=0.1))

    # # Move robot with stored pose
    # r.move(Ptp(goal=pose_after_relative, vel_scale=0.2))

    # # Repeat the previous steps with a sequence command
    # sequence = Sequence()
    # sequence.append(Lin(goal=Pose(position=Point(0.2, 0, 0.8)),
    #                     vel_scale=0.1, acc_scale=0.1))
    # sequence.append(Circ(goal=Pose(position=Point(0.2, -0.2, 0.8)),
    #                      center=Point(0.1, -0.1, 0.8), acc_scale=0.1))
    # sequence.append(Ptp(goal=pose_after_relative, vel_scale=0.2))


    # # Move to start goal for sequence demonstration
    # r.move(Ptp(goal=start_joint_values))

    # # Blend sequence
    # blend_sequence = Sequence()
    # blend_sequence.append(
    #     Lin(goal=Pose(position=Point(0.2, 0, 0.7)), acc_scale=0.05), blend_radius=0.01)
    # blend_sequence.append(
    #     Lin(goal=Pose(position=Point(0.2, 0.1, 0.7)), acc_scale=0.05))

    # r.move(blend_sequence)

    # # Move with custom reference frame
    # r.move(Ptp(goal=PoseStamped(header=Header(frame_id="prbt_tcp"),
    #                             pose=Pose(position=Point(0, 0, 0.1)))))
    # r.move(Ptp(goal=Pose(position=Point(0, -0.1, 0)),
    #            reference_frame="prbt_link_3", relative=True))

    # Create and execute an invalid ptp command with out of bound joint values


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('robot_program_node')

    start_program()
