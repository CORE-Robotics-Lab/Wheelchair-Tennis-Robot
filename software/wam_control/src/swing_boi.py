import rospkg
import json
import ruamel.yaml as yaml
import rospy
import trajectory_msgs.msg as tm
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from numpy import clip, sign
from sensor_msgs.msg import JointState
import time
import copy

catapult_init = {
    "wam/base_yaw_joint": -2.2,
    "wam/shoulder_pitch_joint": -1.57,
    "wam/shoulder_yaw_joint": 1.57,
    "wam/elbow_pitch_joint": 2.63,
    "wam/wrist_yaw_joint": 0,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

catapult_end = {
    "wam/base_yaw_joint": 2.2,
    "wam/shoulder_pitch_joint": -1.57,
    "wam/shoulder_yaw_joint": 1.57,
    "wam/elbow_pitch_joint": -0.78,
    "wam/wrist_yaw_joint": 0,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

catapult_overhead_init = {
    "wam/base_yaw_joint": 0,
    "wam/shoulder_pitch_joint": -1.5708,
    "wam/shoulder_yaw_joint": 0,
    "wam/elbow_pitch_joint": -0.610865,
    "wam/wrist_yaw_joint": 0,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

catapult_overhead_end = {
    "wam/base_yaw_joint": 0,
    "wam/shoulder_pitch_joint": 1.5708,
    "wam/shoulder_yaw_joint": 0,
    "wam/elbow_pitch_joint": 1.5708,
    "wam/wrist_yaw_joint": 0,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}


class FollowTrajectoryClient:
    def __init__(self, joint_limits):
        self.joint_limits = joint_limits
        self.joint_names = joint_limits.keys()
        self.client = actionlib.SimpleActionClient("/joint_group_trajectory_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for joint_group_trajectory_controller...")
        self.client.wait_for_server()
        print("Connected to joint_group_trajectory_controller")

    def get_joint_states(self):
        print("Waiting for joint_states")
        joint_msg = rospy.wait_for_message("/joint_states", JointState, timeout=None)
        print("Got Joint Message")
        curr_joint_position = dict()
        for joint_name, position in zip(joint_msg.name, joint_msg.position):
            curr_joint_position[joint_name] = position
        return curr_joint_position

    # Trapezoidal Position Control
    # Based on github.com/WRidder/MotionProfileGenerator/blob/398754b17f2d92a2f9455dd640e804979403942c/cpp/MotionProfile.cpp  Line 188
    # Really hope this works
    def update_trapezoidal_profile(self, pos, vel, dt, goal_pos, max_vel, max_acc, joint_name, base_joint_vel):
    # def update_trapezoidal_profile(self, pos, vel, dt, goal_pos, max_vel, max_acc):
        # if joint_name == "wam/elbow_pitch_joint":
        #     print(base_joint_vel)

        # You pretty much there
        if abs(goal_pos - pos) < 1e-3:
            return goal_pos, 0

        # Check if we need to de-accelerate [ based on V^2 = V_0^2 + 2A(x-x_0) ]
        if vel**2 /(2*max_acc) >= abs(goal_pos-pos):
            pos += vel * dt - sign(vel) * max_acc * dt**2
            vel += -sign(vel) * max_acc * dt
        else:
            if joint_name == "wam/elbow_pitch_joint":
                if abs(base_joint_vel) >= self.joint_limits["wam/base_yaw_joint"]['max_velocity'] - 0.5:
                    # We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
                    vel = clip(vel + sign(goal_pos - pos) * max_acc * dt, -max_vel, max_vel)
                    pos += vel * dt
                else:
                    pos = pos
                    vel = vel
            else:
                # We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
                vel = clip(vel + sign(goal_pos - pos) * max_acc * dt, -max_vel, max_vel)
                pos += vel * dt
            # # We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
            # vel = clip(vel + sign(goal_pos - pos) * max_acc * dt, -max_vel, max_vel)
            # pos += vel * dt
        return pos, vel

    # Populate trajectory_msgs/JointTrajectory
    def create_trajectory(self, start_joints, end_joints):
        print("Creating Trajectory")

        # Init
        t = 0
        dt = 1e-4
        curr_joints = start_joints.copy()
        trajectory = JointTrajectory()
        curr_velocity = {joint_name: 0 for joint_name in self.joint_names}
        cnt = 0

        while curr_joints != end_joints:
            joints_point = JointTrajectoryPoint()
            joints_point.positions = [0] * len(self.joint_names)
            joints_point.time_from_start = rospy.Duration(t)

            for i, joint_name in enumerate(self.joint_names):
                # Set up
                pos = curr_joints[joint_name]
                vel = curr_velocity[joint_name]
                goal_pos = end_joints[joint_name]
                max_vel = self.joint_limits[joint_name]['max_velocity']
                max_acc = self.joint_limits[joint_name]['max_acceleration']

                # Update
                pos, vel = self.update_trapezoidal_profile(pos, vel, dt, goal_pos, max_vel, max_acc, joint_name, curr_velocity["wam/base_yaw_joint"])
                # pos, vel = self.update_trapezoidal_profile(pos, vel, dt, goal_pos, max_vel, max_acc)

                # Assign
                curr_joints[joint_name] = pos
                curr_velocity[joint_name] = vel

                joints_point.positions[i] = pos

            if cnt % 10 == 0:
                trajectory.points.append(joints_point)

            t += dt
            cnt += 1

        # Create Msg
        trajectory.joint_names = list(self.joint_names)
        print("Trajectory Duration", t, "sec")

        return trajectory

    def send_trajectory(self, trajectory):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        print("Sending Message")
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()
        #print(self.client.get_result())

        return self.client.get_result()

if __name__ == "__main__":
    rospy.init_node('trajectory_follower')

    # Load joint limits
    path = rospkg.RosPack().get_path('wam_moveit') + "/config/joint_limits.yaml"
    with open(path, 'r') as file:
        joint_limits = yaml.safe_load(file)["joint_limits"]

    # Create client
    followClient = FollowTrajectoryClient(joint_limits)

    # Create trajectory and send
    for _ in range(10):
        curr_joint_position = followClient.get_joint_states()
        trajectory = followClient.create_trajectory(curr_joint_position, catapult_init)
        result = followClient.send_trajectory(trajectory)

        time.sleep(3)

        trajectory = followClient.create_trajectory(catapult_init, catapult_end)
        result = followClient.send_trajectory(trajectory)
