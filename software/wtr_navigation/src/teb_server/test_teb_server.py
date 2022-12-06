#!/usr/bin/env python
from __future__ import print_function
import rospy
import actionlib

from wtr_navigation.srv import *
from teb_local_planner.msg import *

import actionlib_msgs.msg
import genpy
import geometry_msgs.msg
import std_msgs.msg
import tf
import random
from tf.transformations import quaternion_from_euler
import math
from visualization_msgs.msg import Marker
import visualization_msgs.msg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Quaternion


import time
import numpy as np


def main():
    rospy.init_node("test_teb_server", anonymous=True)

    print("[Planner] Waiting for Teb server...")
    rospy.wait_for_service('/teb_server/make_plan')
    print("[Planner] Connected to Teb server...")
    teb_plan = rospy.ServiceProxy('/teb_server/make_plan', TebPlans)

    start = geometry_msgs.msg.Pose(orientation=geometry_msgs.msg.Quaternion(w=1))

    goals = []
    for _ in range(100):
        random_pose = geometry_msgs.msg.PoseStamped(header=Header(stamp=rospy.Time.now() + rospy.Duration(100)), pose=geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=random.random()*5-2.5, y=random.random()*5-2.5),
            orientation=geometry_msgs.msg.Quaternion(*quaternion_from_euler(0, 0, random.random()*math.pi))))
        goals.append(random_pose)

    # Publish Pointers
    marker_pub = rospy.Publisher("/candidate_points", Marker, queue_size=1) # Create in init
    marker = Marker(header=Header(frame_id="world"), type=visualization_msgs.msg.Marker.POINTS, pose=Pose(orientation=Quaternion(w=1)),
                    points=[p.pose.position for p in goals], scale=Vector3(.1, .1, .1), color=ColorRGBA(1, 0, 0, 1))

    # This is only to make sure the marker is actually published, publishers sometimes miss the first 1-2 msgs
    # In real code it is fine if first packets go missing since you are publishing at fixed rate
    for i in range(10):
        marker_pub.publish(marker)
        time.sleep(0.1)

    try:
        begin = time.time()
        response = teb_plan(start, goals)
        print("[Planner] Response time:", time.time() - begin)
        print("[Planner] Teb Plan Result:", response.success)
    except rospy.ServiceException as e:
        print("[Planner] Service call failed: %s" % e)
        return

    exec_time = response.best_trajectory.trajectory[-1].time_from_start.to_sec() # sometimes (but rarely) negative... No clue why... Maybe because processing random points
    print("Execution Time", exec_time)
    print(response.best_trajectory.trajectory)

    # Teb had visualization tools for trajectory (but in C++)

if __name__ == '__main__':
    main()
