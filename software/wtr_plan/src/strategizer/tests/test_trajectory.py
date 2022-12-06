import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, '/home/matthew/catkin_ws/src/Wheelchair-Tennis-Robot/Section_3-Control_and_Path_Planning/code')
import tennis
from planner import TebPlanner

TIMESTEP = 0.05


def plot():
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    initial_pos_bounce = tennis.pos(0, 0, 2)
    initial_pos_no_bounce = tennis.pos(-0.27, 1.86, 2)
    planner = TebPlanner()

    b1 = (10, np.radians(182.34), np.radians(-340))
    b2 = (-10, np.radians(33.45), np.radians(0))
    b3 = (3.23, np.radians(0), np.radians(0))
    b4 = (3, np.radians(90), np.radians(-90))
    b5 = (0, np.radians(90), np.radians(90))
    b6 = (10, np.radians(115), np.radians(-76.3))
    b7 = (10, np.radians(-23), np.radians(33))

    traj1 = test_trajectory(b1[0], b1[1], b1[2], initial_pos_no_bounce, planner)
    traj2 = test_trajectory(b2[0], b2[1], b2[2], initial_pos_no_bounce, planner)
    traj3 = test_trajectory(b3[0], b3[1], b3[2], initial_pos_no_bounce, planner)
    traj4 = test_trajectory(b4[0], b4[1], b4[2], initial_pos_no_bounce, planner)
    traj5 = test_trajectory(b5[0], b5[1], b5[2], initial_pos_no_bounce, planner)
    traj6 = test_trajectory(b6[0], b6[1], b6[2], initial_pos_no_bounce, planner)
    traj7 = test_trajectory(b7[0], b7[1], b7[2], initial_pos_bounce, True, planner)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    ax.plot(traj1[0], traj1[1], traj1[2], label="Test1")
    ax.plot(traj2[0], traj2[1], traj2[2], label="Test2")
    ax.plot(traj3[0], traj3[1], traj3[2], label="Test3")
    ax.plot(traj4[0], traj4[1], traj4[2], label="Test4")
    ax.plot(traj5[0], traj5[1], traj5[2], label="Test5")
    ax.plot(traj6[0], traj6[1], traj6[2], label="Test6")
    ax.plot(traj7[0], traj7[1], traj7[2], label="Test7")

    ax.legend()

    plt.show()
    

def test_trajectory(vel, face_angle, ground_angle, initial_pos, planner,
        has_bounced=False):
    ball = tennis.tennis(vel, face_angle, ground_angle, initial_pos, planner,
        has_bounced=has_bounced)

    ball_before_bounce = ball.ball_before_bounce
    ball_after_bounce = ball.ball_after_bounce

    before_bounce_t = ball_before_bounce.getTravelTime() if not has_bounced else None
    after_bounce_t = ball_after_bounce.getTravelTime()

    before_bounce_drop = ball_before_bounce.getDropPoint() if not has_bounced else None
    after_bounce_drop = ball_after_bounce.getDropPoint()

    pos_x = []
    pos_y = []
    pos_z = []

    if not has_bounced:
        for t in np.arange(0, before_bounce_t, TIMESTEP):
            pos_x.append(ball_before_bounce.getXpos(t))
            pos_y.append(ball_before_bounce.getYpos(t))
            pos_z.append(ball_before_bounce.getHeight(t))

        pos_x.append(before_bounce_drop.x)
        pos_y.append(before_bounce_drop.y)
        pos_z.append(before_bounce_drop.z)

    for t in np.arange(0, after_bounce_t, TIMESTEP):
        pos_x.append(ball_after_bounce.getXpos(t))
        pos_y.append(ball_after_bounce.getYpos(t))
        pos_z.append(ball_after_bounce.getHeight(t))

    pos_x.append(after_bounce_drop.x)
    pos_y.append(after_bounce_drop.y)
    pos_z.append(after_bounce_drop.z)

    return pos_x, pos_y, pos_z


plot()
