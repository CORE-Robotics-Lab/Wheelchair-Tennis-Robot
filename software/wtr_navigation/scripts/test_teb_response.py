import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from numpy import clip, sign
import actionlib
import move_base_msgs.msg
from geometry_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler

desired_position = None
vis_pub = None
client = None

def desired_position_callback(msg):
    global desired_position
    desired_position = msg.data

    vis_desired_pos = PointStamped(header=Header(frame_id="world"), point=Point(desired_position, 0., 0.))
    vis_pub.publish(vis_desired_pos)

    goal_pose =  Pose(Point(desired_position, 0, 0), Quaternion(*quaternion_from_euler(0, 0, 0)))   # goal pose
    goal = move_base_msgs.msg.MoveBaseActionGoal(goal=move_base_msgs.msg.MoveBaseGoal(target_pose=PoseStamped(
                                                         header=Header(frame_id="world"), pose=goal_pose)))
    client.send_goal(goal) 
        

def main():
    global vis_pub, client

    rospy.init_node("pid_wheels_script", anonymous=True)

    # Get Robot's desired position
    rospy.Subscriber("/wheelchair_desired_position", Float64,
                     desired_position_callback, queue_size=1)

    # publish a visualization message to compare performance
    vis_pub = rospy.Publisher("/vis_start_move", PointStamped, queue_size=1)

    client = actionlib.SimpleActionClient(
        'move_base', move_base_msgs.msg.MoveBaseAction)
    print("[Planner] Waiting for move_base server...")
    client.wait_for_server()
    print("[Planner] Connected to move_base server...")   

    rospy.spin()

if __name__ == '__main__':
    main()