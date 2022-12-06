#!/usr/bin/env python3
from __future__ import print_function

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from numpy import clip
import collections
import actionlib
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
from wam_control.srv import *
from sensor_msgs.msg import JointState
import time
from statistics import mean

X_THRESHOLD_CAMS = 15

INTERCEPTION_POINT_X = 5 #5.5  #medium speed
# INTERCEPTION_POINT_X = 5 # human. Old: #6 #1.25 #5.0 #3.5
SHOTTYPE = "forehand"
HOLD_RANGE = 0.01

towards_wheelchair = False
client = None
goal_pose = None
hit_pub = None
swing_server = None
wc_angle = None
init_y_wc = None

trust_rollout = False

#y prediction
y_odom_hist = collections.deque(maxlen=10) #[]
y_odom_timestamp = collections.deque(maxlen=10) #[]
y_straight_line = []
before_bounce = True
# y_pose_1_count = 0
# y_odom_hist_pose_1 = []
# y_odom_timestamp_pose_1 = []


def ball_reset_callback(reset_msg):
    # print("reset launch")
    global ball_reset, time_since_reset
    ball_reset = True
    time_since_reset = rospy.get_rostime()

def ball_launch_callback(reset_msg):
    global odom_y, y_odom_hist, y_straight_line, y_odom_timestamp, y_pose_1_count, y_odom_timestamp_pose_1, y_odom_hist_pose_1
    odom_y = None
    y_odom_hist = collections.deque(maxlen=10) #[]
    y_straight_line = []
    y_odom_timestamp = collections.deque(maxlen=10) #[]
    y_pose_1_count = 0
    y_odom_timestamp_pose_1 = []
    y_odom_hist_pose_1 = []
    if reset_msg.data == "Launch":
        global ball_reset, time_since_reset
        ball_reset = True
        time_since_reset = rospy.get_rostime()

def spawn_random_ball(event):
    # print("Spawn ball")
    vy = np.random.normal(0.0, 0.3)
    vx = np.random.normal(-6, 0)
    vz = np.random.normal(7.5, 0.1)
    # Spawn Random Ball
    # pose = Pose(position=Point(1.2, -11, 2.5), orientation=Quaternion(w=1))
    pose = Pose(position=Point(18, 0, 0.3), orientation=Quaternion(w=1))
    pub_cmd.publish(Odometry(header=Header(frame_id="world"),
                             pose=PoseWithCovariance(pose=pose),
                             twist=TwistWithCovariance(
                                 twist=Twist(linear=Vector3(vx, vy, vz)))
                             ))
    time.sleep(.01)
    pub_traj_reset.publish(String("reset"))
    pub_localize_reset.publish(PoseWithCovarianceStamped(
        header=Header(frame_id="world",
                      stamp=rospy.Time.now()),
        pose=PoseWithCovariance(pose=pose)))

def ball_odometry(odometry_msg):
    global towards_wheelchair, before_bounce, odom_y, y_odom_hist, y_straight_line, y_odom_timestamp
    
    if odometry_msg.twist.twist.linear.x < 0:
        towards_wheelchair = True
    else:
        towards_wheelchair = False

    if odometry_msg.twist.twist.linear.z < 0:
        before_bounce = True
    else:
        before_bounce = False

    # if trust_rollout:
    #     odom_y = odometry_msg.pose.pose.position.y
    #     y_odom_timestamp.append(odometry_msg.header.stamp.to_sec())
    #     y_odom_hist.append(odom_y)

    #     if len(y_odom_hist) >= 5:
    #         y_straight_line = np.polyfit(y_odom_timestamp, y_odom_hist, deg=1)

def ball_1_pose_callback(pose_1_msg):
    global odom_y, y_odom_hist, y_odom_timestamp
    if pose_1_msg.pose.position.x < 13:
        odom_y = pose_1_msg.pose.position.y
        y_odom_timestamp.append(pose_1_msg.header.stamp.to_sec())
        y_odom_hist.append(odom_y)
   
        
    # if len(y_odom_hist) >= 3:
    #     y_straight_line = np.polyfit(y_odom_timestamp, y_odom_hist, deg=1)

def ball_2_pose_callback(pose_2_msg):
    pass

def ball_4_pose_callback(pose_4_msg):
    pass

def ball_5_pose_callback(pose_5_msg):
    global odom_y, y_odom_hist, y_odom_timestamp
    if pose_5_msg.pose.position.x > 10 and pose_5_msg.pose.position.x < 15:
        odom_y = pose_5_msg.pose.position.y
        y_odom_timestamp.append(pose_5_msg.header.stamp.to_sec())
        y_odom_hist.append(odom_y)

def ball_6_pose_callback(pose_6_msg):
    global odom_y, y_odom_hist, y_odom_timestamp
    if pose_6_msg.pose.position.x > 10 and pose_6_msg.pose.position.x < 15:
        odom_y = pose_6_msg.pose.position.y
        y_odom_timestamp.append(pose_6_msg.header.stamp.to_sec())
        y_odom_hist.append(odom_y)

    # if len(y_odom_hist) >= 3:
    #     y_straight_line = np.polyfit(y_odom_timestamp, y_odom_hist, deg=1)



def within_geometric_constraints(x, y, z):
    return x > INTERCEPTION_POINT_X and x < (INTERCEPTION_POINT_X + 0.01) and y > -4.11 and y < 4.11

def wc_ready_callback(msg):
    global goal_pose
    if msg.data:
        if wc_angle:
            goal_pose =  Pose(Point(INTERCEPTION_POINT_X, init_y_wc, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))   # goal pose
            # goal_pose =  Pose(Point(INTERCEPTION_POINT_X-1, 2, 0), Quaternion(*quaternion_from_euler(0, 0, 0)))   # goal pose
            goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                                header=Header(frame_id="world"), pose=goal_pose)))
            client.send_goal(goal)
    

def trajectory_rollout_callback(data):
    global goal_pose, hit_pub, swing_server, client, trust_rollout, y_straight_line

    ball_position = None

    trust_rollout = data.poses[0].header.seq

    if towards_wheelchair:        
        if trust_rollout:
            # print("Confidence in Rollout")
            trajectory_poses = data.poses

            for pose in trajectory_poses:
                ball_x = pose.pose.position.x
                ball_y = pose.pose.position.y
                ball_z = pose.pose.position.z
                valid = within_geometric_constraints(ball_x, ball_y, ball_z)
                if valid:
                    ball_position = Point()
                    ball_position.x = pose.pose.position.x
                    ball_position.y = -100 #pose.pose.position.y # dummy value to indicate we don't have a good estimate of y
                    ball_position.z = pose.pose.position.z
                    ball_time_arrival = pose.header.stamp + rospy.Duration(0.13) #0.09

                    print("ball_time_arrival: ")
                    print(ball_time_arrival)
                    print("pose stamp: ")
                    print(pose.header.stamp)

                    if len(y_odom_hist) >= 4:
                        y_straight_line = np.polyfit(y_odom_timestamp, y_odom_hist, deg=1)

                    if len(y_straight_line) == 2:
                        pred_y = y_straight_line[1] + y_straight_line[0] * ball_time_arrival.to_sec()
                        ball_position.y = pred_y
                        # pred_y = mean(y_odom_hist)
                        # ball_position.y = pred_y

                    break

            if ball_position == None or ball_position.y == -100:
                return

            ball_ps = PointStamped(header=Header(frame_id="world", stamp=ball_time_arrival), point=ball_position)
            hit_pub.publish(ball_ps)

            swing_request = SwingRequest()
            swing_request.ball_position = ball_ps
            
            MAGIC = -0.35 #-0.45 #-0.15 #-0.3

            try:
                resp = swing_server(swing_request)
                print(resp)
                if resp.success == True:
                    if abs(goal_pose.position.y - resp.placement_pose.position.y) > HOLD_RANGE and (-2.5 <= resp.placement_pose.position.y + MAGIC <= 2.5):
                        goal_pose = Pose(Point(INTERCEPTION_POINT_X, resp.placement_pose.position.y + MAGIC, 0), resp.placement_pose.orientation)

                        goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                                    header=Header(frame_id="world"), pose=goal_pose)))
                        client.send_goal(goal)
                    pass

                    
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
            
if __name__ == "__main__":    
    rospy.init_node("wc_ball_intercept", anonymous=True)

    # Get Ball Position
    rospy.Subscriber("/ball/rollout/path", Path,
                     trajectory_rollout_callback, queue_size=1)

    rospy.Subscriber("/ball_odometry", Odometry,
                     ball_odometry, queue_size=1)

    rospy.Subscriber("/ball/event", String,
                     ball_launch_callback, queue_size=1)

    rospy.Subscriber("/ball_1_pose_transformed", PoseStamped,
                     ball_1_pose_callback, queue_size=1)

    # rospy.Subscriber("/ball_5_pose_transformed", PoseStamped,
    #                  ball_5_pose_callback, queue_size=1)

    # rospy.Subscriber("/ball_6_pose_transformed", PoseStamped,
    #                  ball_6_pose_callback, queue_size=1)

    SIM = False
    if SIM:
        # Command Ball Position
        pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

        # Reset
        pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
        pub_localize_reset = rospy.Publisher("/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

        # Spawn ball on timer
        rospy.Subscriber("/ball/set_pose", PoseWithCovarianceStamped, ball_reset_callback, queue_size=1)

        spawn_ball_timer = rospy.Timer(rospy.Duration(10), spawn_random_ball)
    
    # Hit Ball Position
    hit_pub = rospy.Publisher("/hit_ball_point", PointStamped, queue_size=1)

    # subscribe to ready call back function
    rospy.Subscriber("/wc_getready", Bool, wc_ready_callback, queue_size=1)


    # Command move_base Goal
    client = actionlib.SimpleActionClient(
        'move_base', move_base_msgs.msg.MoveBaseAction)
    print("[Planner] Waiting for move_base server...")
    client.wait_for_server()
    print("[Planner] Connected to move_base server...")

    print("[Planner] Waiting for swing server...")
    rospy.wait_for_service('swing_server')
    swing_server = rospy.ServiceProxy('/swing_server', Swing)
    print("[Planner] Connected to swing server...")

    

    if SHOTTYPE == "forehand":
        wc_angle = -1.571
        init_y_wc = 0 #-0.5 #-1.0
    elif SHOTTYPE == "backhand":
        wc_angle = 1.571
        init_y_wc = -1
    else:
        raise Exception("Shot type should be backhand or forehand")
    
    
    goal_pose =  Pose(Point(INTERCEPTION_POINT_X, init_y_wc, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))   # goal pose
    goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                         header=Header(frame_id="world"), pose=goal_pose)))
    client.send_goal(goal)

    rospy.spin()

