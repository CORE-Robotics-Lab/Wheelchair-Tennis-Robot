from __future__ import print_function

import rospy
from wtr_navigation.srv import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker
from std_msgs.msg import *

from visualization_msgs.msg import Marker

# dynamics of the tennis ball and court
RHO_H = 0.05        # Horizontal coefficient of restitution in the range [0, 0.24]
RHO_V = 0.75        # Vertical coefficient of restitution in the range [0.728, 0.759]
ALPHA = 2/5         # Uniform sphere (2/3 for thin hollow sphere)

# These are the coordinate limits in meters that robot can move to
# notice that the tennis is the tennis after hitting ground
# Note these bounds go slightly beyond the court to ensure the robot pursues
# hard to call balls
MAX_HIT_X = 6.0
MIN_HIT_X = -6.0
MAX_HIT_Y = 11
MIN_HIT_Y = -5
MAX_HIT_Z = 1.25
MIN_HIT_Z = 0.8

dt = 3.3e-3   # change in time for hit position calculations


'''Garwin's Model for bounce prediction.

Assumptions:
    - no spin
    - constant azimuth

Reference:
    https://aapt.scitation.org/doi/pdf/10.1119/1.1450571
'''
def ball_after_bounce(vx, vy, vz):
    vx = ((1 - ALPHA * RHO_H) * vx) / (1 + ALPHA)
    vy = ((1 - ALPHA * RHO_H) * vy) / (1 + ALPHA)
    vz = -RHO_V * vz

    return vx, vy, vz


class Tennis:
    def __init__(self, ball_odometry, bounces):
        self.trajectory_pub = rospy.Publisher("/ball_trajectory", Marker, queue_size=1)
        self.update(ball_odometry, bounces)


    def update(self, ball_odometry=None, bounces=None):
        if ball_odometry is not None:
            self.odometry = ball_odometry
            self.pos = ball_odometry.pose.pose.position
            self.vel = ball_odometry.twist.twist.linear

        if bounces is not None:
            self.bounces = bounces

        self.time = rospy.Time.now()

        if ball_odometry is not None or bounces is not None:
            self.get_future_path()

    def within_geometric_constraints(self, x, y, z):
        return (
            x > MIN_HIT_X and x < MAX_HIT_X and
            y > MIN_HIT_Y and y < MAX_HIT_Y and
            z > MIN_HIT_Z and z < MAX_HIT_Z)

    def get_future_path(self):
        bounces = self.bounces
        x, y, z = self.pos.x, self.pos.y, self.pos.z
        vx, vy, vz = self.vel.x, self.vel.y, self.vel.z

        trajectory = []
        legal_trajectory = []
        t = 0

        while bounces < 2:
            in_bounds = self.within_geometric_constraints(x, y, z)
            time = self.time + rospy.Duration(t)

            trajectory.append([x, y, z, time, in_bounds])

            if in_bounds:
                legal_trajectory.append([x, y, z, time])

            vz += -9.8 * dt

            if z + vz * dt <= 0:
                vx, vy, vz = ball_after_bounce(vx, vy, vz)
                bounces += 1

            x += vx * dt
            y += vy * dt
            z += vz * dt
            t += dt

        self.trajectory = trajectory
        self.legal_trajectory = legal_trajectory

    def display_trajectory(self):
        trajectory = self.trajectory
        marker = Marker(
            header=Header(frame_id="world"),
            type=Marker.LINE_STRIP,
            pose=Pose(orientation=Quaternion(w=1)),
            points=[Point(p[0], p[1], p[2]) for p in trajectory],
            scale=Vector3(.1, .1, .1),
            color=ColorRGBA(0,1,0,1))

        self.trajectory_pub.publish(marker)
