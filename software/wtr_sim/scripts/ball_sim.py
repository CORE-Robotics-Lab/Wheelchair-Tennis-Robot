#!/usr/bin/env python3
"""
This is a script to control the trajectory of the tennis ball
in the Gazebo world. This scripts can be restructured into
a Gazebo Model pluggin, but a python script would be easier
to edit for future uses. The physics of the ball of very limited
currently.
"""

from __future__ import print_function

import time
import sys

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist, Point, PoseStamped
from std_msgs.msg import Header
import numpy as np
import random

from visualization_msgs.msg import Marker
import visualization_msgs.msg
from std_msgs.msg import ColorRGBA, Header

from geometry_msgs.msg import *

import tf

# Global Variables
viz_ball_pub = None
racquet_pub = None
listener = None
bounce = 0
odometry = Odometry()

FLOOR = 0.033  # Ball Radius
noise = False

# Rate
physics_loop_rate = 300
publish_estimate_rate = 50
publish_truth_rate = 100

# dynamics of the tennis ball and court
RHO_H = 0.05    # Horizontal coefficient of restitution in the range [0, 0.24]
RHO_V = 0.75    # Vertical coefficient of restitution in the range [0.728, 0.759]
ALPHA = 2 / 5   # Uniform sphere (2/3 for thin hollow sphere)

# Delay
delay_mean = 0.000
delay_std = 0.000

history_buffer = []


# In case you want to command new pose and velocity
def update_odometry(odometry_msg):
    print("[Ball Trajectory] Ball Command Recieved")
    global odometry, bounce
    odometry = odometry_msg
    bounce = 0


def check_racquet_collision():
    # Transform to racquet point of view
    global odometry
    try:
        poseStamped = listener.transformPose('wam/racquet_hitpoint_link',
                                             PoseStamped(Header(frame_id="world", stamp=rospy.Time(0)), odometry.pose.pose))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        # Maybe haven't started publishing the tf yet
        return

    # Visualize Box
    width, height, depth = .3, .3, 0.08  # Kinda arbitrary
    racquet_pub.publish(Marker(header=Header(frame_id="wam/racquet_hitpoint_link"), type=visualization_msgs.msg.Marker.CUBE,
                               pose=Pose(orientation=Quaternion(w=1)), scale=Vector3(width, height, depth), color=ColorRGBA(0, 0, 1, .8)))

    pos = poseStamped.pose.position

    # Check if hit
    if (-width/2 < pos.x and pos.x < width/2) \
            and (-height/2 < pos.y and pos.y < height/2) \
            and (-depth/2 < pos.z and pos.z < depth/2):
        # Hit!
        # Transform linear velocity
        # Assume world and ball orientation is the same since we aren't publishing ball tf
        v3s = listener.transformVector3('wam/racquet_hitpoint_link',
                                        Vector3Stamped(Header(frame_id="world", stamp=rospy.Time(0)), odometry.twist.twist.linear))

        v3s.vector.z *= -1

        new_v3s = listener.transformVector3('world',
                                            Vector3Stamped(Header(frame_id="wam/racquet_hitpoint_link", stamp=rospy.Time(0)), v3s.vector))

        odometry.twist.twist.linear = new_v3s.vector
        # TODO consider speed of racquet


def camera_publishing_estimate(camera_link, publisher, point, time_taken):
    # Get relative ball
    global odometry
    try:
        poseStamped = listener.transformPose(camera_link, PoseStamped(
            Header(frame_id="world", stamp=rospy.Time(0)), Pose(position=point)))
        # print(poseStamped)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        return

    pose = poseStamped.pose
    # pose.orientation = Quaternion(w=1)  # Don't know orientation

    # Add Noise

    pose.position.x += np.random.normal(0, 0.03)
    pose.position.y += np.random.normal(0, 0.03)
    pose.position.z += np.random.normal(0, 0.05)

    covariance = np.diagflat([3e-2, 3e-2, 5e-2, 0, 0, 0]).flatten().tolist()

    publisher.publish(PoseWithCovarianceStamped(
        header=Header(frame_id=camera_link,
                      stamp=time_taken),
        pose=PoseWithCovariance(pose=pose, covariance=covariance)))


def publish_truth(event):
    viz_ball_pub.publish(odometry)

def main():
    rospy.init_node("ball_trajectory", anonymous=True)
    global racquet_pub, viz_ball_pub

    rospy.Subscriber("/ball_cmd", Odometry, update_odometry, queue_size=1)
    viz_ball_pub = rospy.Publisher(
        '/ball_truth_odometry', Odometry, queue_size=1)

    racquet_pub = rospy.Publisher("/racquet_collision", Marker, queue_size=1)

    camera_1_pub = rospy.Publisher(
        "/ball_1_pose", PoseWithCovarianceStamped, queue_size=1)
    camera_2_pub = rospy.Publisher(
        "/ball_2_pose", PoseWithCovarianceStamped, queue_size=1)
    camera_3_pub = rospy.Publisher(
        "/ball_3_pose", PoseWithCovarianceStamped, queue_size=1)
    camera_4_pub = rospy.Publisher(
        "/ball_4_pose", PoseWithCovarianceStamped, queue_size=1)

    global listener
    listener = tf.TransformListener()

    global odometry, bounce
    odometry.header.frame_id = "world"
    odometry.child_frame_id = "ball"
    odometry.pose.pose.position = Point(4, 4, FLOOR)
    odometry.pose.pose.orientation = Quaternion(w=1)

    rospy.Timer(rospy.Duration(1.0 / publish_truth_rate), publish_truth)

    curr_time = rospy.Time.now()
    last_time = rospy.Time.now()

    last_estimate_time = rospy.Time.now()

    rate = rospy.Rate(physics_loop_rate)
    while not rospy.is_shutdown():
        # This likely can be replaced by a class's update function to make cleaner
        pos = odometry.pose.pose.position
        # assume in global frame (not realtive to ball)
        v = odometry.twist.twist.linear

        curr_time = rospy.Time.now()
        dt = (curr_time - last_time).to_sec()

        # v.x, v.y don't need to be updated because only worrying about gravity for now
        v.z += -9.8 * dt

        if pos.z + v.z * dt <= FLOOR:
            if noise:
                rho_z = np.random.normal(RHO_V, 0.05)
                rho_x = np.random.normal(RHO_H, 0.05)
                rho_y = np.random.normal(RHO_H, 0.05)

                vx = ((1 - ALPHA * RHO_H) * vx) / (1 + ALPHA)
                vy = ((1 - ALPHA * RHO_H) * vy) / (1 + ALPHA)

                v.x = ((1 - ALPHA * rho_x) * v.x) / (1 + ALPHA)
                v.y = ((1 - ALPHA * rho_y) * v.y) / (1 + ALPHA)
                v.z = rho_z * abs(v.z)
            else:
                v.z = .75 * abs(v.z)
            bounce += 1

        pos.x += v.x * dt
        pos.y += v.y * dt
        pos.z += v.z * dt

        last_time = curr_time

        check_racquet_collision()
        odometry.header.stamp = curr_time

        if (curr_time - last_estimate_time) >= rospy.Duration(1 / publish_estimate_rate):
            last_estimate_time = curr_time

            point = Point(pos.x, pos.y, pos.z)
            time_taken = curr_time
            delay = rospy.Duration(np.random.normal(delay_mean, delay_std))
            history_buffer.append([point, time_taken, delay])

        # Publish from zed with noise
        if len(history_buffer) > 0 and curr_time >= history_buffer[0][1] + history_buffer[0][2]:
            point, time_taken, delay = history_buffer.pop(0)
            if pos.x <= 12:
                camera_publishing_estimate("camera_1_center_camera_optical", camera_1_pub, point, time_taken)
                camera_publishing_estimate("camera_2_center_camera_optical", camera_2_pub, point, time_taken)
            else:
                camera_publishing_estimate("camera_3_center_camera_optical", camera_3_pub, point, time_taken)
                camera_publishing_estimate("camera_4_center_camera_optical", camera_4_pub, point, time_taken)

        rate.sleep()


if __name__ == '__main__':
    main()
