from __future__ import print_function

import random
import sys

from teb_local_planner.msg import *

sys.path.insert(1, '/home/matthew/catkin_ws/src/Wheelchair-Tennis-Robot/Section_3-Control_and_Path_Planning/code')
sys.path.insert(1, '/home/matthew/catkin_ws/src/Wheelchair-Tennis-Robot/Section_3-Control_and_Path_Planning/code/planners')


import teb_planner

def test():
    start_pos = geometry_msgs.msg.Pose(orientation=geometry_msgs.msg.Quaternion(w=1))

    goals = []

    for _ in range(100):
        random_pose = geometry_msgs.msg.Pose(
            position=geometry_msgs.msg.Point(x=random.random()*5, y=random.random()*5),
            orientation=geometry_msgs.msg.Quaternion(w=1))
        goals.append(random_pose)

    plan = teb_planner.best_path(start_pos, goals, debug=True)

    print(plan)


if __name__ == '__main__':
    test()
