import pathlib
path_to_file = pathlib.Path(__file__).parent.resolve()

import math

import numpy as np
import rospy
import visualization_msgs.msg
from wtr_navigation.srv import *
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped,
                               Quaternion, Vector3)
from nav_msgs.msg import Odometry
from rospy import Publisher
from std_msgs.msg import ColorRGBA, Header
from tf.listener import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

from planner import Planner
from tennis import Tennis

# Swing Types
SWING = ['LEFT_LINEAR', 'RIGHT_LINEAR']
# SWING = ['LEFT_LINEAR', 'RIGHT_LINEAR', 'LEFT_CIRCULAR', 'RIGHT_CIRCULAR']


# Orientations the robot can be at to hit the ball
HIT_ANGLE = [1/8 * math.pi, 0, -1/8 * math.pi]
# HIT_ANGLE = [0]

# Default position for the robot
DEFAULT_ROBOT_POSE = Pose(
    position=Point(0, 0, 0),
    orientation=Quaternion(*quaternion_from_euler(0, 0, 0))
)

NCANDIDATE_POINTS = 100
HOLD_RANGE = 0.5       # When a point on the trajectory is within this distance of the previous hit pos hold position
BUFFER_TIME = rospy.Duration(secs=0)

HIT_GOAL = Vector3(15, 0, 0)

def swing_offset(hit_angle, swing, z, swing_type='linear'):
    if swing_type == 'linear':
        if hit_angle == HIT_ANGLE[0]:
            offset = (-0.1, 1.0999) if z < 1.1 else (0, 0.8999)
        elif hit_angle == HIT_ANGLE[1]: # TODO: If add other swings make elif and 1
            offset = (0.1, -1.0999) if z < 0.9 else (0.1, -1.0999)
        elif hit_angle == HIT_ANGLE[2]:
            offset = (0.1, -1.1999) if z < 0.9 else (0.1, -1.0999)
    elif swing_type == 'circular':
        # TODO: Circular swing offset
        if hit_angle == HIT_ANGLE[0]:
            return None
        elif hit_angle == HIT_ANGLE[1]:
            offset = (0, 1.5)
        elif hit_angle == HIT_ANGLE[2]:
            return None

    if swing == SWING[1]:
        offset = (offset[0], -offset[1])

    return offset

X = 2 * len(HIT_ANGLE)

def distance(x: Pose, y: Pose):
    x = np.array([x.position.x, x.position.y, 0])
    y = np.array([y.position.x, y.position.y, 0])
    dist = np.linalg.norm(x - y, ord=2)

    return dist


'''Plan hit positions and paths using the given strategy.

Initialize with current data. When you get new data call update() to update
information. To get the x, y, z position to hit the ball at call get_hit_pos().
To get a path to the hit position call get_path_to_hit_pos().

Arguments:
    robot_odometry: state of the robot
    ball_odometry: state of the ball
    planner: path planner for robot
    bounce_state: amount of bounces done by the ball
    ball_trajectory_pub: published for displaying predicted trajectory of ball

Example:
    strat = strategizer(args)

    path = get_path_to_hit_pos()
    robot.move(path)

    obs = ekf_state()

    strat.update(obs)

    path = get_path_to_hit_pos()
    robot.move(path)

TODO:
    This is a basic strategizer in order to advance to path planning, eventually this
    should be replaced with a more powerful strategizer. The basic approach here is a
    linear solver, but a reinforcement learning agent will be able to perform better and
    make deeper decisions.
'''
class strategizer:
    def __init__(self, robot_odometry: Odometry, ball_odometry: Odometry,
            planner: Planner, bounce_state=0, swing_type='linear'):
        self.robot_odometry = robot_odometry
        self.tennis = Tennis(ball_odometry, bounce_state)
        self.planner = planner
        self.swing_type = swing_type

        self.goal_poses = []
        self.hit_ball = None
        self.prev_goal_pos = None
        self.relative_ball_pnt = None
        self.last_path_time = rospy.get_rostime()

    '''Update state of robot and ball'''
    def update(self, robot_odometry, ball_odometry=None, bounces=None):
        self.robot_odometry = robot_odometry
        self.tennis.update(ball_odometry, bounces)

    '''Get the best path to hit the ball

    Return:
        (goal position, (ball position, time)) or None

        if goal position is None then the goal position has not changed
    '''
    def get_hit_path(self):
        valid_traj = np.array(self.tennis.legal_trajectory)

        goal_pose_stamps = []
        self.goal_poses = []
        pose_distance = np.array([])

        if self.prev_goal_pos is not None:
            prev_quat = self.prev_goal_pos.orientation
            quat = [prev_quat.x, prev_quat.y, prev_quat.z, prev_quat.w]
            prev_h = euler_from_quaternion(quat)[2]

        for i, p in enumerate(valid_traj):
            for h in HIT_ANGLE:
                pose_stamps = []

                for s in SWING:
                    offset = self.get_swing_offset(h, s, p[2])

                    if offset is None:
                        continue

                    x_offset, y_offset = offset

                    pose = PoseStamped(
                        header=Header(stamp=p[3] - BUFFER_TIME),
                        pose=Pose(
                            position=Point(p[0] + x_offset, p[1] + y_offset, p[2]),
                            orientation=Quaternion(*quaternion_from_euler(0, 0, h))
                        ))

                    pose_stamps.append(pose)

                    # If previous path is valid update ball position but stay on path
                    if self.prev_goal_pos is not None:
                        dst_to_ball = distance(pose.pose, self.prev_goal_pos)
                        pose_distance = np.append(pose_distance, dst_to_ball)

                        if dst_to_ball <= HOLD_RANGE and np.isclose(h, prev_h):
                            self.hit_ball = p
                            ball_stamp = PointStamped(
                                header=Header(frame_id="world"),
                                point=Point(p[0], p[1], p[2]))

                            relative_ball_pnt = self.ball_robot_transform(
                                self.prev_goal_pos.position, self.prev_goal_pos.orientation, ball_stamp.point)

                            self.relative_ball_pnt = relative_ball_pnt

                            self.last_path_time = rospy.get_rostime()

                            return None, (relative_ball_pnt, p[3])

                poses = [pose.pose for pose in pose_stamps]
                self.goal_poses.extend(poses)
                goal_pose_stamps.extend(pose_stamps)

        if len(goal_pose_stamps) == 0:
            return None

        # Only use closest points
        goal_pose_stamps = np.array(goal_pose_stamps)

        if len(goal_pose_stamps) > NCANDIDATE_POINTS:
        #     if self.prev_goal_pos is not None:
        #         pos_idx = np.argsort(pose_distance)[:NCANDIDATE_POINTS]
        #         goal_pose_stamps = goal_pose_stamps[pos_idx]
        #     else:
            goal_pose_stamps = np.random.choice(goal_pose_stamps, size=NCANDIDATE_POINTS, replace=False)

        # Get path if possible plan
        response = self.planner.best_path(
            self.robot_odometry.pose.pose, goal_pose_stamps)

        if response.success:
            goal_pose = response.best_trajectory.trajectory[-1].pose
            ball = valid_traj[response.best_index // X] # // X Depends how many goals we add per ball position
            self.hit_ball = ball

            ball_stamp = PointStamped(header=Header(frame_id="world"), point=Point(ball[0], ball[1], ball[2]))
            relative_ball_pnt = self.ball_robot_transform(
                goal_pose.position, goal_pose.orientation, ball_stamp.point)
            self.relative_ball_pnt = relative_ball_pnt

            self.prev_goal_pos = goal_pose
            self.last_path_time = rospy.get_rostime()

            return goal_pose, (relative_ball_pnt, ball[3])

        # TODO: Default position
        return None

    '''Get direction to swing

    Arguments:
        ball: location of the ball
        orientation: quaternion orientation of the robot (if None then last pose by strategizer)

    Returns:
        direction vector to swing in
    '''
    def swing_direction(self, ball, orientation=None):
        if orientation is None:
            orientation = self.prev_goal_pos.orientation

        # Get robot euler angles
        orientation = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        # Get direction relative to robot
        direction = Vector3(
            HIT_GOAL.x - ball.x + 12,
            HIT_GOAL.y - ball.y,
            0,
        )

        direction.z = direction.x + direction.y

        # print("[strategizer:swing_direction] direction vector", direction)

        # Rotate direction relative to world
        theta = -orientation[2]

        direction.x = direction.x * np.cos(theta) - direction.y * np.sin(theta)
        direction.y = direction.x * np.sin(theta) + direction.y * np.cos(theta)

        return direction

    '''Position Relative to Future Wheelchair Position'''
    def ball_robot_transform(self, base_position, base_orientation, ball_position):
        # Fun transform math
        # From tf.listner's fromTranslationRotation, asMatrix, transformPoint
        base_translation = base_position.x, base_position.y, base_position.z
        base_rotation = base_orientation.x, base_orientation.y, base_orientation.z, base_orientation.w
        ball_x, ball_y, ball_z = ball_position.x, ball_position.y, ball_position.z

        mat44 = np.dot(transformations.translation_matrix(base_translation), transformations.quaternion_matrix(base_rotation))
        mat44 = np.linalg.inv(mat44)
        xyz = tuple(np.dot(mat44, np.array([ball_x, ball_y, ball_z, 1.0])))[:3]

        return Point(*xyz)

    '''Get the offset of the robot to hit the ball with the given approach
    angle and swing type

    f(a_theta, swing) => (x_offset, y_offset)

    Return:
        (x_offset, y_offset)
    '''
    def get_swing_offset(self, a, swing, z):
        # return SWING_OFFSET[(a, swing)]
        return swing_offset(a, swing, z, swing_type=self.swing_type)

    def return_to_default(self):
        if self.tennis.ball_odometry.twist.twist.linear.x > 0:
            return DEFAULT_ROBOT_POSE

        if rospy.get_rostime() - self.last_path_time > rospy.Duration(secs=5):
            return DEFAULT_ROBOT_POSE

        return None

    def display_goals(self, marker_pub):
        marker = Marker(
            header=Header(frame_id="world"),
            type=visualization_msgs.msg.Marker.POINTS,
            pose=Pose(orientation=Quaternion(w=1)),
            points=[Point(p.position.x, p.position.y, .05) for p in self.goal_poses],
            scale=Vector3(.1, .1, .1),
            color=ColorRGBA(0, 0, 1, 1))

        marker_pub.publish(marker)

    def display_ball(self, hit_pub):
        if self.hit_ball is not None:
            ball = self.hit_ball
            ball = PointStamped(
                header=Header(frame_id="world"),
                point=Point(ball[0], ball[1], ball[2]))
            hit_pub.publish(ball)

    def display_ball_trajectory(self, ball_trajectory_pub):
        trajectory = self.tennis.trajectory
        marker = Marker(
            header=Header(frame_id="world"),
            type=visualization_msgs.msg.Marker.LINE_STRIP,
            pose=Pose(orientation=Quaternion(w=1)),
            points=[Point(p[0], p[1], p[2]) for p in trajectory],
            scale=Vector3(.1, .1, .1),
            colors=[ColorRGBA(1*(not p[4]), 1*p[4], 0, 1) for p in trajectory])

        ball_trajectory_pub.publish(marker)
