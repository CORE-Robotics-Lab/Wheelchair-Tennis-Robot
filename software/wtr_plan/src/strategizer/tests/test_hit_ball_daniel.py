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
from barrett_tennis.srv import *
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from tf.listener import transformations


robot_state = None
pub_cmd = None
pub_cmd = None
pub_traj_reset = None
pub_localize_reset = None
strat = None
client = None
hit_pos = None


bounce_state = 0    # 0 = pre-bounce, 1 = post-bounce
last_velz = None

robot_odometry = None
ball_odometry = None
trajectory_pub = None
teb_plan = None
once = True
candidate_pub = None
listener = None
hit_pub = None
linear_swing = None

ball_x = None
linear_move = None

goal = None
goal_pose_time = None


class Ball:
    def __init__(self, ball_odometry, bounces):
        self.odometry = ball_odometry
        self.bounces = bounces
        self.pos = ball_odometry.pose.pose.position
        self.vel = ball_odometry.twist.twist.linear
        self.time = rospy.Time.now()
        self.get_future_path()

    def within_geometric_constraints(self, x, y, z):
        # Only deal with position constraints, not time (that will be dealt by teb_server)
        return x > -3 and x < 12 and z > .9 and z < 1.4

    def get_future_path(self):
        bounces = self.bounces
        x, y, z = self.pos.x, self.pos.y, self.pos.z
        vx, vy, vz = self.vel.x, self.vel.y, self.vel.z

        trajectory = []
        dt = 0.02
        t = 0

        while bounces < 2:
            valid = self.within_geometric_constraints(x, y, z)
            time = self.time + rospy.Duration(t)

            trajectory.append([x, y, z, time, valid])

            vz += -9.8 * dt

            if z + vz * dt <= 0:
                vz = .5 * abs(vz)
                bounces += 1

            x += vx * dt
            y += vy * dt
            z += vz * dt
            t += dt

        self.trajectory = trajectory


# ---- End of Class ----


def update_bounce_state(velz, x):
    global last_velz,  bounce_state

    if x > 20:
        last_velz = velz
        bounce_state = 0
        return

    if last_velz == None:
        last_velz = velz
        return

    if last_velz < 0 and velz >= 0:  # Direction Change
        bounce_state += 1

    last_velz = velz


def observation_callback(odometry_msg):
    global ball_odometry

    # Position is always in world, velocity is in child frame
    try:
        world_linear = listener.transformVector3("world", Vector3Stamped(Header(
            frame_id=odometry_msg.child_frame_id, stamp=rospy.Time(0)), odometry_msg.twist.twist.linear))
        odometry_msg.twist.twist.linear = world_linear.vector
    except (tf.LookupException, AttributeError):
        return

    ball_odometry = odometry_msg

    update_bounce_state(world_linear.vector.z,
                        odometry_msg.pose.pose.position.x)


def robot_callback(odometry_msg):
    global robot_odometry
    robot_odometry = odometry_msg


def recalculate_goal(event):
    #print("===================")
    global goal
    if goal != None:
        start = time.perf_counter()
        #print("#############")
        goal_poses_refined = []
        offset_angles = np.linspace(0.0, 6.28, 50)
        for h in offset_angles:
            orientation_quat = Quaternion(*quaternion_from_euler(0, 0, h))
            pose = PoseStamped(header=Header(stamp=rospy.Time.now() + rospy.Duration(100)),
                               pose=Pose(goal.goal.target_pose.pose.position,
                                         Quaternion(*quaternion_from_euler(0, 0, h))))
            goal_poses_refined.append(pose)
        #print(goal_poses_refined)

        response_refined = teb_plan(
            robot_odometry.pose.pose, goal_poses_refined)
        #print("response_refined: ")
        #print(response_refined)
        if response_refined.success:
            goal_pose = response_refined.best_trajectory.trajectory[-1].pose
            #print("Goal Pose after replanning: ")
            #print(goal_pose)
            #This should maybe be done by another node

            # Just send the goal position, ball position, time else where
            goal = move_base_msgs.msg.MoveBaseActionGoal(
                goal=move_base_msgs.msg.MoveBaseGoal(
                    target_pose=PoseStamped(
                        header=Header(frame_id="world"),
                        pose=goal_pose)))

            client.send_goal(goal)  # Don't wait

        end = time.perf_counter()
        time_diff = end - start
        print(f'Took {time_diff} seconds to execute the recalculate goals loop')


def calculate_ball(event):
    global goal

    if not ball_odometry:
        return
    ball = Ball(ball_odometry, bounce_state)
    trajectory = ball.trajectory

    marker = Marker(header=Header(frame_id="world"), type=visualization_msgs.msg.Marker.LINE_STRIP, pose=Pose(orientation=Quaternion(w=1)),
                    points=[Point(p[0], p[1], p[2]) for p in trajectory], scale=Vector3(.1, .1, .1), colors=[ColorRGBA(1*(not p[4]), 1*p[4], 0, 1) for p in trajectory])
    trajectory_pub.publish(marker)

    valid_traj = list(filter(lambda x: x[4], trajectory))  # Get Only valid
    # lol = list(valid_traj)

    goal_poses = []

    for p in valid_traj:
        # Lets just do left and right offset for now, ugly for now
        # Left:
        for h in [1.571]:
            pose_1 = PoseStamped(header=Header(stamp=p[3]),
                                 pose=Pose(Point(p[0], p[1]-1, 0), Quaternion(*quaternion_from_euler(0, 0, h))))
            pose_2 = PoseStamped(header=Header(stamp=p[3]),
                                 pose=Pose(Point(p[0], p[1]+1, 0), Quaternion(*quaternion_from_euler(0, 0, h))))
            goal_poses.append(pose_1)
            goal_poses.append(pose_2)

        # Right:
        # for h in [0, -np.pi/2]:
        #     pose = PoseStamped(
        #         header=Header(stamp=p[3]),
        #         pose=Pose(Point(p[0], p[1]+1, 0), Quaternion(*quaternion_from_euler(0, 0, h))))
        #     goal_poses.append(pose)

        # See if similar enough to last time

    # Show candidate points
    marker = Marker(header=Header(frame_id="world"), type=visualization_msgs.msg.Marker.POINTS, pose=Pose(orientation=Quaternion(w=1)),
                    points=[p.pose.position for p in goal_poses], scale=Vector3(.1, .1, .1), color=ColorRGBA(0, 0, 1, 1))
    candidate_pub.publish(marker)

    global once, goal_pose_time
    if once:
        # Get if possible Teb planner
        response = teb_plan(robot_odometry.pose.pose, goal_poses)

        # Go there
        if response.success:
            once = False
            goal_pose = response.best_trajectory.trajectory[-1].pose

            # Just send the goal position, ball position, time else where
            goal = move_base_msgs.msg.MoveBaseActionGoal(
                goal=move_base_msgs.msg.MoveBaseGoal(
                    target_pose=PoseStamped(
                        header=Header(frame_id="world"),
                        pose=goal_pose)))

            client.send_goal(goal)  # Don't wait

            # Find Ball
            #ball = valid_traj[response.best_index // 1] # // X Depends how many goals we add per ball position
            # // X Depends how many goals we add per ball position
            ball = valid_traj[response.best_index // 2]
            t = ball[3]
            ball = PointStamped(header=Header(frame_id="world"),
                                point=Point(ball[0], ball[1], ball[2]))
            hit_pub.publish(ball)

            # Position Relative to Future Wheelchair Position
            def transform(base_position, base_orientation, ball_position):
                # Fun transform math
                # From tf.listner's fromTranslationRotation, asMatrix, transformPoint
                base_translation = base_position.x, base_position.y, base_position.z
                base_rotation = base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w
                ball_x, ball_y, ball_z = ball_position.x, ball_position.y, ball_position.z

                mat44 = np.dot(transformations.translation_matrix(
                    base_translation), transformations.quaternion_matrix(base_rotation))
                mat44 = np.linalg.inv(mat44)
                xyz = tuple(np.dot(mat44, np.array(
                    [ball_x, ball_y, ball_z, 1.0])))[:3]

                return Point(*xyz)

            relative_ball_pnt = transform(
                goal_pose.position, goal_pose.orientation, ball.point)
            global ball_x
            ball_x = PointStamped(header=Header(
                frame_id="base_footprint", stamp=t), point=relative_ball_pnt)

            #response = linear_move(ball_x)
            # if (t > rospy.Time.now()):
            #     time.sleep((t - rospy.Time.now() - rospy.Duration(.4)).to_sec())
            # else:
            #     return

            # Call first part

            # If at ball and racquet in position, swing, (just swing regardless for now)
            # try:
            #     response = linear_swing(ball_x)
            #     print(response)
            #     # print("sucess")
            #     # print(ball_x)
            # except rospy.ServiceException as e:
            #     pass
            #     # print("[Planner] Swing failed: %s" % e)
            #     # print(ball_x)

        else:
            pass
            # goal_pose = Pose(orientation=Quaternion(w=1))  # Just go home


def spawn_random_ball(event):
    vx = random.random() * -2 - 8
    vy = random.random() * 1 - 1
    vz = random.random() * 1 + 8

    # Spawn Random Ball
    pose = Pose(position=Point(25, 1, 2), orientation=Quaternion(w=1))
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

    global once, goal
    once = True
    goal = None


def main():
    global pub_cmd, bounce_state, client, pub_traj_reset, pub_localize_reset
    rospy.init_node("test_ball_traj", anonymous=True)

    # Get Ball Position
    rospy.Subscriber("/ball_odometry", Odometry,
                     observation_callback, queue_size=1)

    # Get Robot Position
    rospy.Subscriber("/odometry/filtered", Odometry,
                     robot_callback, queue_size=1)

    # Command Ball Position
    pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

    # Reset
    pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
    pub_localize_reset = rospy.Publisher(
        "/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

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

    # Make Ball Trajectory
    global trajectory_pub
    trajectory_pub = rospy.Publisher("/ball_trajectory", Marker, queue_size=1)

    # Candidate Points
    global candidate_pub
    candidate_pub = rospy.Publisher("/candidate_points", Marker, queue_size=1)

    # Hit Ball
    global hit_pub
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # Transform
    global listener
    listener = tf.TransformListener()

    # Linear Swing
    global linear_move, linear_swing
    print("[Planner] Waiting for LinearSwing server...")
    rospy.wait_for_service('/linear_stroke_move')
    rospy.wait_for_service('/linear_stroke_swing')
    # print("[Planner] Connected to LinearSwing server...")
    # linear_move = rospy.ServiceProxy('/linear_stroke_move', LinearSwing)
    # linear_swing = rospy.ServiceProxy('/linear_stroke_swing', LinearSwing)

    # Timer updates
    publish_state_timer = rospy.Timer(rospy.Duration(10.0), spawn_random_ball)
    publish_state_timer = rospy.Timer(rospy.Duration(0.2), calculate_ball)
    publish_state_timer = rospy.Timer(rospy.Duration(0.1), recalculate_goal)

    rospy.spin()


if __name__ == '__main__':
    main()
