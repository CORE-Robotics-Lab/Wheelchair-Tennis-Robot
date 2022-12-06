#!/usr/bin/env python3
from __future__ import print_function

from visualization_msgs.msg import Marker
import visualization_msgs.msg
from std_msgs.msg import ColorRGBA, Header
import random
import sys
import time

import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import numpy as np
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from wtr_navigation.srv import *
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from tf.listener import transformations

HOLD_RANGE = 0.1

towards_wheelchair = False
candidate_pub = None
teb_plan = None
client = None
robot_odometry = None
listener = None
goal_pose = None

def correct_angle(angle):
    if (angle < 0):
        return angle + 2 * math.pi
    if (angle > 2 * math.pi):
        return angle - 2 * math.pi
    return angle
    
def angle_abs_diff(angle_one, angle_two):
    diff = abs(angle_one - angle_two)
    
    if (diff > math.pi):
        diff = 2 * math.pi - diff
    
    return diff

def distance(x: Pose, y: Pose):
    x = np.array([x.position.x, x.position.y, 0])
    y = np.array([y.position.x, y.position.y, 0])
    dist = np.linalg.norm(x - y, ord=2)

    return dist

def ball_odometry(odometry_msg):
    global towards_wheelchair

    if odometry_msg.twist.twist.linear.x < 0:
        towards_wheelchair = True
    else:
        towards_wheelchair = False

def within_geometric_constraints(x, y, z):
        # return x > -3 and x < 5 and z > .9 and z < 1.4
        return x > 3 and x < 3.01

def trajectory_rollout_callback(data):
    global robot_odometry, goal_pose, prev_goal_pose

    if towards_wheelchair:
        valid_traj = []
        goal_poses = []
        if data.poses[0].header.seq == 1:
            # print("Confidence in Rollout")
            trajectory_poses = data.poses

            for pose in trajectory_poses:
                valid = within_geometric_constraints(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
                if valid:
                    valid_traj.append(pose)

            for p in valid_traj:
                for h in [1.571]:
                    pose_1 = PoseStamped(header=Header(stamp=p.header.stamp + rospy.Duration(1000000)),
                                 pose=Pose(Point(p.pose.position.x, p.pose.position.y-1, 0), Quaternion(*quaternion_from_euler(0, 0, h))))
                    # pose_2 = PoseStamped(header=Header(stamp=p.header.stamp + rospy.Duration(1000000)),
                    #              pose=Pose(Point(p.pose.position.x, p.pose.position.y+1, 0), Quaternion(*quaternion_from_euler(0, 0, h))))
                    goal_poses.append(pose_1)
                    # goal_poses.append(pose_2)

            # See if similar enough to last time

            # Show candidate points
            marker = Marker(header=Header(frame_id="world"), type=visualization_msgs.msg.Marker.POINTS, pose=Pose(orientation=Quaternion(w=1)),
                            points=[p.pose.position for p in goal_poses], scale=Vector3(.1, .1, .1), color=ColorRGBA(0, 0, 1, 1))
            candidate_pub.publish(marker)

            
            if len(goal_poses) > 0:
                # Get if possible Teb planner
                # looking up position of base_footprint in world frame of reference
                (trans,rot) = listener.lookupTransform('/world', '/base_footprint', rospy.Time(0))
                robot_pose = Pose()
                robot_pose.position.x = trans[0]
                robot_pose.position.y = trans[1]
                robot_pose.position.z = trans[2]
                robot_pose.orientation.x = rot[0]
                robot_pose.orientation.y = rot[1]
                robot_pose.orientation.z = rot[2]
                robot_pose.orientation.w = rot[3]

                response = teb_plan(robot_pose, goal_poses)

                if response.success:
                    curr_goal_pose = response.best_trajectory.trajectory[-1].pose
                    if goal_pose != None:
                        dist_from_prev_goal = distance(goal_pose, curr_goal_pose)
                        if dist_from_prev_goal >= HOLD_RANGE:
                            goal_pose = curr_goal_pose
                    else:
                        goal_pose = curr_goal_pose

                if (goal_pose != None):
                    # pointing_vector = [robot_pose.position.x - goal_pose.position.x, robot_pose.position.y - goal_pose.position.y, 0]
                    # pointing_angle = correct_angle(math.atan2(pointing_vector[1], pointing_vector[0]))
                    # alt_pointing_angle = correct_angle(pointing_angle - math.pi)
                    # if (angle_abs_diff(rot[2], pointing_angle) > angle_abs_diff(rot[2], alt_pointing_angle)):
                    #     pointing_angle = alt_pointing_angle
                    # goal_pose =  Pose(Point(goal_pose.position.x, goal_pose.position.y, 0), Quaternion(*quaternion_from_euler(0, 0, pointing_angle)))
                    
                    # Just send the goal position, ball position, time else where
                    goal = move_base_msgs.msg.MoveBaseActionGoal(
                        goal=move_base_msgs.msg.MoveBaseGoal(
                            target_pose=PoseStamped(
                                header=Header(frame_id="world"),
                                pose=goal_pose)))

                    client.send_goal(goal)  # Don't wait

                # print("Goal Pose: ")
                # print(goal_pose)

        #print(valid_traj)
        #print(len(valid_traj))
        


def main():
    global client
    global listener

    rospy.init_node("wc_ball_intercept", anonymous=True)

    # Get Ball Position
    rospy.Subscriber("/ball/rollout/path", Path,
                     trajectory_rollout_callback, queue_size=1)

    rospy.Subscriber("/ball_odometry", Odometry,
                     ball_odometry, queue_size=1)

    #transform istener
    listener = tf.TransformListener()
    
    # Candidate Points
    global candidate_pub
    candidate_pub = rospy.Publisher("/candidate_points", Marker, queue_size=1)

    # Command move_base Goal
    client = actionlib.SimpleActionClient(
        'move_base', move_base_msgs.msg.MoveBaseAction)
    print("[Planner] Waiting for move_base server...")
    client.wait_for_server()
    print("[Planner] Connected to move_base server...")
    
    # Contact Teb server
    rospy.wait_for_service('/teb_server/make_plan')
    print('[Planner] Connected to Teb Planner...')
    global teb_plan
    teb_plan = rospy.ServiceProxy('/teb_server/make_plan', TebPlans)

    
    # # Get Robot Position
    # rospy.Subscriber("/odometry/filtered", Odometry,
    #                  robot_callback, queue_size=1)

    # # Command Ball Position
    # pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

    # # Reset
    # pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
    # pub_localize_reset = rospy.Publisher(
    #     "/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

    # # Command move_base Goal
    # client = actionlib.SimpleActionClient(
    #     'move_base', move_base_msgs.msg.MoveBaseAction)
    # print("[Planner] Waiting for move_base server...")
    # client.wait_for_server()
    # print("[Planner] Connected to move_base server...")

    # # Contact Teb server
    # rospy.wait_for_service('/teb_server/make_plan')
    # print('[Planner] Connected to Teb Planner...')
    # global teb_plan
    # teb_plan = rospy.ServiceProxy('/teb_server/make_plan', TebPlans)

    # # Make Ball Trajectory
    # # global trajectory_pub
    # # trajectory_pub = rospy.Publisher("/ball_trajectory", Marker, queue_size=1)

    # # Candidate Points
    # global candidate_pub
    # candidate_pub = rospy.Publisher("/candidate_points", Marker, queue_size=1)

    # # Hit Ball
    # global hit_pub
    # hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # # Transform
    # global listener
    # listener = tf.TransformListener()

    # # Linear Swing
    # # global linear_move, linear_swing
    # # print("[Planner] Waiting for LinearSwing server...")
    # # rospy.wait_for_service('/linear_stroke_move')
    # # rospy.wait_for_service('/linear_stroke_swing')
    # # print("[Planner] Connected to LinearSwing server...")
    # # linear_move = rospy.ServiceProxy('/linear_stroke_move', LinearSwing)
    # # linear_swing = rospy.ServiceProxy('/linear_stroke_swing', LinearSwing)

    # # Timer updates
    # publish_state_timer = rospy.Timer(rospy.Duration(10.0), spawn_random_ball)
    # publish_state_timer = rospy.Timer(rospy.Duration(0.2), calculate_ball)
    # publish_state_timer = rospy.Timer(rospy.Duration(0.1), recalculate_goal)

    rospy.spin()


if __name__ == '__main__':
    main()
