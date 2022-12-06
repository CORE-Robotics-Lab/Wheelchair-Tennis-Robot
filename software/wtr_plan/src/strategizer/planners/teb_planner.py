from __future__ import print_function

import geometry_msgs.msg
import rospy
import visualization_msgs.msg
from wtr_navigation.srv import *
from geometry_msgs.msg import Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from teb_local_planner.msg import *
from utils import debug_msg
from visualization_msgs.msg import Marker

MSG_SRC = '[Teb Planner]'


teb_make_plan = None

'''Get teb plan'''
def spin_up(init_node=False):
    global teb_make_plan, marker_pub
    print('[Planner] Connecting to Teb Planner...')

    if init_node:
        rospy.init_node('teb_planner_server', anonymous=True)
    rospy.wait_for_service('/teb_server/make_plan')

    print('[Planner] Connected to Teb Planner...')

    teb_make_plan = rospy.ServiceProxy('/teb_server/make_plan', TebPlans)


'''Find the optimal path based on the TebPlanner.

Optimal path is path of least time.

Arguments:
    start_pos: Pose of robot
    goal_pos: list of Pos for locations to hit the ball

Returns:
    Teb Plan Response holding the best path
'''
def best_path(start_pos: geometry_msgs.msg.Pose, goal_pos: list, debug=False):
    try:
        response = teb_make_plan(start_pos, goal_pos)
    except rospy.ServiceException as e:
        debug_msg(MSG_SRC, 'Service call failed: %s' % e)
        return

    return response
