#!/usr/bin/env python3
from __future__ import print_function

from defer import return_value

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from numpy import clip
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
import collections

X_THRESHOLD_CAMS = 18

INTERCEPTION_POINT_X = 5.4 #4.5 #5.4 #4.7
init_y_wc = None
wc_angle = None
SHOTTYPE = "forehand"
HOLD_RANGE = 0.02

SLOPE = 1.2 #1.05

towards_wheelchair = False
client = None
goal_pose = None
hit_pub = None
swing_server = None

#y averaging
y_cam_hist = collections.deque(maxlen=7) #[]
y_cam_timestamp = collections.deque(maxlen=7) #[]
y_straight_line = collections.deque(maxlen=7) #[]
start_time = None
before_bounce = True

def ball_reset_callback(reset_msg):
    # print("reset launch")
    global ball_reset, time_since_reset
    ball_reset = True
    time_since_reset = rospy.get_rostime()

def ball_launch_callback(reset_msg):
    global y_cam_hist, y_cam_timestamp, y_straight_line, before_bounce, start_time, goal_pose
    if reset_msg.data == "Launch":
        global ball_reset, time_since_reset
        ball_reset = True
        time_since_reset = rospy.get_rostime()

        y_cam_hist = collections.deque(maxlen=7) #[]
        y_cam_timestamp = collections.deque(maxlen=7) #[]
        y_straight_line = collections.deque(maxlen=7) #[]

        before_bounce = True

        start_time = rospy.Time.from_sec(time.time()).to_sec()

        offset = -1.1 #-0.8 #-0.6 #-0.5 #-0.2 #-0.7
        goal_pose =  Pose(Point(INTERCEPTION_POINT_X, init_y_wc+offset, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))   # goal pose
        goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                            header=Header(frame_id="world"), pose=goal_pose)))
        client.send_goal(goal)

def lateral_cam_1(posestampedmessage):
    if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def lateral_cam_2(posestampedmessage):
    if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def lateral_cam_3(posestampedmessage):
    if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def lateral_cam_4(posestampedmessage):
    if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def lateral_cam_5(posestampedmessage):
    if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def lateral_cam_6(posestampedmessage):
    # if posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
    if 11 <= posestampedmessage.pose.position.x <= X_THRESHOLD_CAMS:
        y_cam_hist.append(posestampedmessage.pose.position.y)
        y_cam_timestamp.append(posestampedmessage.header.stamp.to_sec() - start_time)

def spawn_random_ball(event):
    # print("Spawn ball")
    vy = np.random.normal(0.0, 0.1)
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

def wc_ready_callback(msg):
    global goal_pose
    if msg.data:
        goal_pose =  Pose(Point(INTERCEPTION_POINT_X, init_y_wc, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))   # goal pose
        # goal_pose =  Pose(Point(INTERCEPTION_POINT_X-1, init_y_wc+0.7, 0), Quaternion(*quaternion_from_euler(0, 0, 0)))   # goal pose
        goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                            header=Header(frame_id="world"), pose=goal_pose)))
        client.send_goal(goal)



def ball_odometry(odometry_msg):
    global towards_wheelchair, before_bounce
    
    if odometry_msg.twist.twist.linear.x < 0:
        towards_wheelchair = True
    else:
        towards_wheelchair = False

    if odometry_msg.twist.twist.linear.z < 0:
        before_bounce = True
    else:
        before_bounce = False

def within_geometric_constraints(x, y, z):
    return x > INTERCEPTION_POINT_X and x < (INTERCEPTION_POINT_X + 0.01) and y > -4.5 and y < 4.5 and 0.838 <= z <= 2.0

def trajectory_rollout_callback(data):
    global goal_pose, hit_pub, swing_server, client, y_straight_line

    ball_position = None

    if towards_wheelchair and before_bounce:        
        if data.poses[0].header.seq:
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
                    ball_position.y = -100 # dummy value to indicate we don't have a good estimate of y
                    ball_position.z = pose.pose.position.z
                    ball_time_arrival = pose.header.stamp

                    if len(y_cam_hist) >= 4:
                        y_straight_line = np.polyfit(y_cam_timestamp, y_cam_hist, deg=1)
                    if len(y_straight_line) == 2:
                        # pred_y = y_straight_line[1] + y_straight_line[0] * ball_time_arrival.to_sec()
                        pred_y = y_straight_line[1] + y_straight_line[0] * (ball_time_arrival.to_sec()-start_time) * SLOPE
                        # print("pred y = " + str(pred_y))
                        ball_position.y = pred_y
                        # pred_y = mean(y_odom_hist)
                        # ball_position.y = pred_y


                    # ball_position = pose.pose.position
                    # ball_time_arrival = pose.header.stamp
                    

                    break

            if ball_position == None or ball_position.y == -100:
                return

            ball_ps = PointStamped(header=Header(frame_id="world", stamp=ball_time_arrival), point=ball_position)
            hit_pub.publish(ball_ps)

            swing_request = SwingRequest()
            ball_position_send = PointStamped()
            ball_position_send.header.stamp = ball_ps.header.stamp
            ball_position_send.point.x = ball_ps.point.x
            ball_position_send.point.y = ball_ps.point.y
            ball_position_send.point.z = ball_ps.point.z
            swing_request.ball_position = ball_position_send
            
            MAGIC = 0.2 #-0.3 #0.0 #-0.5 #-0.36

            # print("===============1")

            try:
                resp = swing_server(swing_request)
                # print(resp)
                # print("===============2")
                if resp.success == True:
                    if abs(goal_pose.position.y - resp.placement_pose.position.y) > HOLD_RANGE:
                        y_pos = resp.placement_pose.position.y + MAGIC
                        if y_pos < -3.5:
                            y_pos = -3.5
                        elif y_pos > 3.5:
                            y_pos = 3.5
                        # print(y_pos)
                        # if -1.5 < x_pos < 0.0:
                        # if -3.5 < y_pos < 3.5:
                        if -3.5 < y_pos < 1.5:
                            # print("hereee")
                            goal_pose = Pose(Point(INTERCEPTION_POINT_X, y_pos, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))
                            goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                                        header=Header(frame_id="world"), pose=goal_pose)))
                            client.send_goal(goal)
                            # print("===============3")

                    
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

    # rospy.Subscriber("/ball_1_pose_transformed", PoseStamped,
    #                  lateral_cam_1, queue_size=1)

    # rospy.Subscriber("/ball_2_pose_transformed", PoseStamped,
    #                  lateral_cam_2, queue_size=1)

    # rospy.Subscriber("/ball_3_pose_transformed", PoseStamped,
    #                  lateral_cam_3, queue_size=1)

    # rospy.Subscriber("/ball_4_pose_transformed", PoseStamped,
    #                  lateral_cam_4, queue_size=1)

    rospy.Subscriber("/ball_5_pose_transformed", PoseStamped,
                     lateral_cam_5, queue_size=1)

    # rospy.Subscriber("/ball_6_pose_transformed", PoseStamped,
    #                  lateral_cam_6, queue_size=1)

    

    SIM = False
    if SIM:
        # Command Ball Position
        pub_cmd = rospy.Publisher("/ball_cmd", Odometry, queue_size=1)

        # Reset
        pub_traj_reset = rospy.Publisher("/syscommand", String, queue_size=1)
        pub_localize_reset = rospy.Publisher("/ball/set_pose", PoseWithCovarianceStamped, queue_size=1)

        # Spawn ball on timer
        rospy.Subscriber("/ball/set_pose", PoseWithCovarianceStamped, ball_reset_callback, queue_size=1)

        spawn_ball_timer = rospy.Timer(rospy.Duration(5), spawn_random_ball)
    
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

    if SHOTTYPE == "forehand":
        wc_angle = -1.571
        init_y_wc = y = 0 #-0.3 #-0.6 #-0.9 #-1.2 #-1.5 #-1.5 #-2 #-1.5
    elif SHOTTYPE == "backhand":
        wc_angle = 1.571
        init_y_wc = 0
    else:
        raise Exception("Shot type should be backhand or forehand")

    goal_pose =  Pose(Point(INTERCEPTION_POINT_X, init_y_wc, 0), Quaternion(*quaternion_from_euler(0, 0, wc_angle)))   # goal pose
    goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                         header=Header(frame_id="world"), pose=goal_pose)))
    client.send_goal(goal)    

    print("[Planner] Waiting for swing server...")
    rospy.wait_for_service('swing_server')
    swing_server = rospy.ServiceProxy('/swing_server', Swing)
    print("[Planner] Connected to swing server...")


    rospy.spin()

