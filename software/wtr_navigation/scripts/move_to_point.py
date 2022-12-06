#!/usr/bin/env python3
import rospy
from wtr_navigation.srv import *
import actionlib
import move_base_msgs.msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
import random
from tf.transformations import quaternion_from_euler
import tf
import math
from threading import Timer

client = None
listener = None
x = 0
y = 0

def robot_callback(odometry_msg):
    # reading current trajectory of robot
    global robot_odometry
    robot_odometry = odometry_msg
    
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
    
def reflect_angle(angle, mirror):
    reflected = (2 * mirror) - angle
    return correct_angle(reflected)
    
def move_to_point(event):
    global x
    global y
    x = random.uniform(0, 7)
    y = random.uniform(-5, 5)

    # looking up position of base_footprint in world frame of reference
    (trans,rot) = listener.lookupTransform('/world', '/base_footprint', rospy.Time(0))
    
    robot_angle = correct_angle(rot[2])
    pointing_vector = [trans[0] - x, trans[1] - y, 0]
    pointing_angle = correct_angle(math.atan2(pointing_vector[1], pointing_vector[0]))
    alt_pointing_angle = correct_angle(pointing_angle - math.pi)
    
    if (angle_abs_diff(robot_angle, pointing_angle) > angle_abs_diff(robot_angle, alt_pointing_angle)):
       pointing_angle = alt_pointing_angle
    
    print("Goal Angle is:")
    print(pointing_angle)
    
    goal_pose =  Pose(Point(x, y, 0), Quaternion(*quaternion_from_euler(0, 0, pointing_angle)))   # goal pose
    goal = move_base_msgs.msg.MoveBaseActionGoal(
        goal=move_base_msgs.msg.MoveBaseGoal(
            target_pose=PoseStamped(
                header=Header(frame_id="world"),
                pose=goal_pose)))

    
    wait_time = 1
    update = Timer(wait_time, recalculate_point)
    update.start()
    client.send_goal(goal)  # Don't wait
    
def recalculate_point():
    goal_state = client.get_goal_status_text()
    
    if goal_state == "Goal reached.":
    	return

    
    # looking up position of base_footprint in world frame of reference
    (trans,rot) = listener.lookupTransform('/world', '/base_footprint', rospy.Time(0))
    
    robot_angle = correct_angle(rot[2])
    pointing_vector = [trans[0] - x, trans[1] - y, 0]
    pointing_angle = correct_angle(math.atan2(pointing_vector[1], pointing_vector[0]))
    alt_pointing_angle = correct_angle(pointing_angle - math.pi)
    
    if (angle_abs_diff(robot_angle, pointing_angle) > angle_abs_diff(robot_angle, alt_pointing_angle)):
       pointing_angle = alt_pointing_angle
    
    print("Updated Goal Angle is:")
    print(pointing_angle)
    
    goal_pose =  Pose(Point(x, y, 0), Quaternion(*quaternion_from_euler(0, 0, pointing_angle)))   # goal pose
    goal = move_base_msgs.msg.MoveBaseActionGoal(
        goal=move_base_msgs.msg.MoveBaseGoal(
            target_pose=PoseStamped(
                header=Header(frame_id="world"),
                pose=goal_pose)))


    wait_time = 1
    update = Timer(wait_time, recalculate_point)
    update.start()
    client.send_goal(goal)  # Don't wait

def main():
    global client
    global listener

    rospy.init_node("move_to_point", anonymous=True)

      # Get Robot Position
    rospy.Subscriber("/odometry/filtered", Odometry,
                     robot_callback, queue_size=1)

    #transform istener
    listener = tf.TransformListener()
    

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

    # Timer updates
    publish_state_timer = rospy.Timer(rospy.Duration(15.0), move_to_point)
    
    # looking up position of base_footprint in world frame of reference
    #(trans,rot) = listener.lookupTransform('/world', '/base_footprint', rospy.Time(0))
    #print("Current robot position and orientation: ")
    #print(trans)
    #print(rot)
    
    

    rospy.spin()

if __name__ == '__main__':
    main()



