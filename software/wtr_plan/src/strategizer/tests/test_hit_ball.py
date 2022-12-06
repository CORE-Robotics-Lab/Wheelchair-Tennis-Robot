#!/usr/bin/env python
from __future__ import print_function

import argparse
import random
import sys

import rospkg

sys.path.insert(1, rospkg.RosPack().get_path('barrett_tennis') + "/src/strategizer/")

import actionlib
import actionlib_msgs.msg
import move_base_msgs.msg
import numpy as np
import rospy
from barrett_tennis.srv import *
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker




from planner import TebPlanner
from strategizer import strategizer
import time

pub_cmd = None
marker_pub = None
pub_traj_reset = None
pub_localize_reset = None
strat = None
client = None
hit_pos = None
swing_type = None

planner = TebPlanner()

bounce_state = 0    # 0 = pre-bounce, 1 = post-bounce
last_velz = None

ball_odometry = None
robot_odometry = None
ball_trajectory_pub = None
hit_pub = None

linear_move = None
linear_swing = None

swing_dir = None


def setup_test():
    rospy.init_node("test_ball_traj", anonymous=True)

    global swing_type
    swing_type = rospy.get_param("~swing_type")

    global pub_cmd, client, ball_trajectory_pub, hit_pub, marker_pub, pub_traj_reset, pub_localize_reset
    global linear_swing, linear_move

    ball_trajectory_pub = rospy.Publisher(
        "/ball_trajectory", Marker, queue_size=1)

    # Get Ball Position
    rospy.Subscriber(
        "/ball_odometry", Odometry, observation_callback, queue_size=1)

    # Get Robot Position
    rospy.Subscriber(
        "/odometry/filtered", Odometry, robot_callback, queue_size=1)

    # Command Ball Position
    pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

    # Hit Ball Position
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # Goal hit position
    marker_pub = rospy.Publisher("/candidate_points", Marker, queue_size=1)

    # Reset
    pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
    pub_localize_reset = rospy.Publisher("/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

    client = actionlib.SimpleActionClient(
        'move_base', move_base_msgs.msg.MoveBaseAction)
    print("[Planner] Waiting for move_base server...")
    client.wait_for_server()
    print("[Planner] Connected to move_base server...")

    print("[Planner] Waiting for LinearSwing server...")
    rospy.wait_for_service('/linear_stroke_move')
    rospy.wait_for_service('/linear_stroke_swing')
    print("[Planner] Connected to LinearSwing server...")
    linear_move = rospy.ServiceProxy('/linear_stroke_move', LinearSwing)
    linear_swing = rospy.ServiceProxy('/linear_stroke_swing', LinearSwing)

    rospy.Timer(rospy.Duration(5.0), spawn_random_ball_callback)
    rospy.Timer(rospy.Duration(0.05), path_callback)

    rospy.spin()

def update_bounce_state(velz, x):
    global last_velz, bounce_state

    if x > 20:
        last_velz = None
        bounce_state = 0
        return

    if last_velz is None:
        last_velz = velz
        return

    if last_velz == 0:
        last_velz = velz
        return

    if last_velz < 0 and velz >= 0:
        bounce_state += 1

    last_velz = velz

def spawn_random_ball_callback(event):
    vx = random.random() * -2 - 8
    vy = random.random() * 3 - 1
    vz = random.random() * 2 + 6

    # Spawn Random Ball
    pose = Pose(position=Point(25, 1, 2), orientation=Quaternion(w=1))
    pub_cmd.publish(Odometry(header=Header(frame_id="world"),
                             pose=PoseWithCovariance(pose=pose),
                             twist=TwistWithCovariance(
                                 twist=Twist(linear=Vector3(vx, vy, vz)))
                             ))
    time.sleep(.1)

    # Reset history path and ekf
    pub_traj_reset.publish(String("reset"))
    pub_localize_reset.publish(PoseWithCovarianceStamped(
        header=Header(frame_id="world",
                      stamp=rospy.Time.now()),
        pose=PoseWithCovariance(pose=pose)))

def path_callback(event):
    global hit_pub, swing_dir
    import time
    start_time = time.time()
    if bounce_state > 1 or strat is None:
        return

    hit_path = strat.get_hit_path()
    # print('took time', time.time() - start_time)
    if hit_path is None:
        return

    goal_pose, (ball, time) = hit_path

    ball_x = PointStamped(header=Header(frame_id="base_footprint"), point=ball)

    if goal_pose is not None:
        goal = move_base_msgs.msg.MoveBaseActionGoal(
            header=Header(frame_id="world"),
            goal_id=actionlib_msgs.msg.GoalID(
                stamp=rospy.get_rostime(), id=0),
            goal=move_base_msgs.msg.MoveBaseGoal(
                target_pose=PoseStamped(
                    header=Header(frame_id="world"),
                    pose=goal_pose)))

        global_ball_pos = Vector3(
            goal_pose.position.x,
            goal_pose.position.y + ball.y,
            goal_pose.position.z + ball.z,
        )
        client.send_goal(goal)  # Don't wait

        swing_dir = strat.swing_direction(global_ball_pos)
        response = linear_move(ball_x, swing_dir)

    if hit_pub is not None:
        strat.display_ball(hit_pub)

    if swing_dir is not None:
        try:
            response = linear_swing(ball_x, swing_dir)
            print('[test hit] percent complete', response.percent_complete)
        except rospy.ServiceException as e:
            pass
    else:
        print('[test hit] swing dir not initialized')

def observation_callback(odometry_msg):
    global ball_odometry, strat
    ball_odometry = odometry_msg

    update_bounce_state(
        odometry_msg.twist.twist.linear.z, odometry_msg.pose.pose.position.x)

    if strat is None:
        strat = strategizer(robot_odometry, ball_odometry, planner,
            bounce_state=bounce_state, swing_type=swing_type)
    else:
        strat.update(robot_odometry, ball_odometry, bounce_state)

    strat.display_ball_trajectory(ball_trajectory_pub)
    strat.display_goals(marker_pub)


def robot_callback(odometry_msg):
    global robot_odometry
    robot_odometry = odometry_msg


def main():
    setup_test()


if __name__ == '__main__':
    main()
