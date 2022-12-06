#!/usr/bin/env python
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

import rospkg
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import moveit_commander
import moveit_msgs.msg
import ruamel.yaml as yaml
from swing_boi import catapult_init, catapult_end, catapult_overhead_init, catapult_overhead_end, FollowTrajectoryClient

# This script looks at joystick button presses and makes WAM go:
#   -   Home by pressing square button
#   -   Zero position by pressing circle button
#   -   Swing position 1 by pressing x button
#   -   Awing position 2 by pressing x button


class MoveGroupInterface(object):
    """MoveGroupInterface"""

    def __init__(self):
        # Initializing moveit stuff
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface_joystick", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_end_effector_link("wam/racquet_hitpoint_link")
        # move_group.set_planning_time(5)
        # move_group.set_planner_id("RRTstarkConfigDefault")

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # setting initial acceleration and velocity scaling factors to 0.1
        acc_scal = 0.1
        vel_scal = 0.1
        move_group.set_max_acceleration_scaling_factor(acc_scal)
        move_group.set_max_velocity_scaling_factor(vel_scal)

        # setting the default height of catapult swing corresponding to a sholder pitch angle of -1.64
        catapult_height = catapult_init["wam/shoulder_pitch_joint"]#-1.64

        # Load joint limits
        path = rospkg.RosPack().get_path('wam_moveit') + "/config/joint_limits.yaml"
        with open(path, 'r') as file:
            joint_limits = yaml.safe_load(file)["joint_limits"]

        # Create client
        self.followClient = FollowTrajectoryClient(joint_limits) 

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.robot_position = None # should be one of None, zero, home, catapult1, catapult2
        self.acc_scal = acc_scal
        self.vel_scal = vel_scal
        self.catapult_height = catapult_height


        # Setting up subscriber for joystick
        rospy.Subscriber("joy", Joy, self.js_callback, queue_size=1)


    def home(self):
        joint_goals = [0.0, -1.98, 0.0, 3.1, 0.0, 0.0, 0.0]
        result = self.move_group.go(joint_goals, wait=True)
        self.move_group.stop()
        if result:
            self.robot_position = 'home'

    def zero(self):
        joint_goals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = self.move_group.go(joint_goals, wait=True)
        self.move_group.stop()
        if result:
            self.robot_position = 'zero'

    def catapult_1(self):
        curr_joint_position = self.followClient.get_joint_states()
        trajectory = self.followClient.create_trajectory(curr_joint_position, catapult_init)
        result = self.followClient.send_trajectory(trajectory)
        result = result.error_code
        if result == 0:
            self.robot_position = 'catapult1'

    def catapult_2(self):
        curr_joint_position = self.followClient.get_joint_states()
        trajectory = self.followClient.create_trajectory(curr_joint_position, catapult_end)
        result = self.followClient.send_trajectory(trajectory)
        result = result.error_code
        if result == 0:
            self.robot_position = 'catapult2'

    # def overhead_catapult_1(self):
    #     curr_joint_position = self.followClient.get_joint_states()
    #     trajectory = self.followClient.create_trajectory(curr_joint_position, catapult_overhead_init)
    #     result = self.followClient.send_trajectory(trajectory)
    #     result = result.error_code
    #     if result == 0:
    #         self.robot_position = 'overhead_catapult1'

    # def overhead_catapult_2(self):
    #     curr_joint_position = self.followClient.get_joint_states()
    #     trajectory = self.followClient.create_trajectory(curr_joint_position, catapult_overhead_end)
    #     result = self.followClient.send_trajectory(trajectory)
    #     result = result.error_code
    #     if result == 0:
    #         self.robot_position = 'overhead_catapult2'

    def old_catapult_1(self):
        joint_goals = [2.30, self.catapult_height, -1.27, 2.39, 0.0, 0.0, 0.0]
        result = self.move_group.go(joint_goals, wait=True)
        self.move_group.stop()
        if result:
           self.robot_position = 'old_catapult1'

    def old_catapult_2(self):
        joint_goals = [-2.30, self.catapult_height, -1.27, -0.523, 0.0, 0.0, 0.0]
        result = self.move_group.go(joint_goals, wait=True)
        self.move_group.stop()
        if result:
            self.robot_position = 'old_catapult2'

    # def spiral_catapult_1(self):
    #     joint_goals = [2.30, self.catapult_height + 1, -1.27, 2.39, 0.0, 0.0, 0.0]
    #     result = self.move_group.go(joint_goals, wait=True)
    #     self.move_group.stop()
    #     if result:
    #         self.robot_position = 'spiral_catapult1'

    # def spiral_catapult_2(self):
    #     joint_goals = [-2.30, self.catapult_height, -1.27, -0.523, 0.0, 0.0, 0.0]
    #     result = self.move_group.go(joint_goals, wait=True)
    #     self.move_group.stop()
    #     if result:
    #         self.robot_position = 'spiral_catapult2'
    
    def increase_vel_acc_scaling(self):
        if (self.vel_scal + 0.1 <= 1.0) and (self.acc_scal + 0.1 <= 1.0):
            self.vel_scal += 0.1
            self.acc_scal += 0.1
            self.move_group.set_max_velocity_scaling_factor(self.vel_scal)
            self.move_group.set_max_acceleration_scaling_factor(self.acc_scal)
            print(f"Increasing velocity and acceleration scaling factor to {round(self.vel_scal, 1)}")

    def decrease_vel_acc_scaling(self):
        if (self.vel_scal - 0.1 >= 0.1) and (self.acc_scal - 0.1 >= 0.1):
            self.vel_scal -= 0.1
            self.acc_scal -= 0.1
            self.move_group.set_max_velocity_scaling_factor(self.vel_scal)
            self.move_group.set_max_acceleration_scaling_factor(self.acc_scal)
            print(f"Decreasing velocity and acceleration scaling factor to {round(self.vel_scal, 1)}")

    def increase_catapult_height(self):
        if self.robot_position in ['catapult1', 'catapult2']:
            if self.catapult_height + 0.02 <= -1:
                self.catapult_height += 0.02
                catapult_init["wam/shoulder_pitch_joint"] = self.catapult_height
                joint_goals = list(self.robot.get_current_state().joint_state.position)
                joint_goals[1] = self.catapult_height
                self.move_group.go(joint_goals, wait=True)
                self.move_group.stop()

    def decrease_catapult_height(self):
        if self.robot_position in ['catapult1', 'catapult2', 'old_catapult1', 'old_catapult2']:
            if self.catapult_height - 0.02 >= -1.98:
                self.catapult_height -= 0.02
                catapult_init["wam/shoulder_pitch_joint"] = self.catapult_height
                joint_goals = list(self.robot.get_current_state().joint_state.position)
                joint_goals[1] = self.catapult_height
                self.move_group.go(joint_goals, wait=True)
                self.move_group.stop()

    def js_callback(self, data):
        cross_button = data.buttons[0]     # new catapult position 1
        circle_button = data.buttons[1]    # zero position
        triangle_button = data.buttons[2]  # new atapult position 2
        square_button = data.buttons[3]    # home position

        vel_acc_scale_inc = data.buttons[5]
        vel_acc_scale_dec = data.buttons[4]

        catapult_height_inc = data.buttons[13]
        catapult_height_dec = data.buttons[14]

        old_catapult_1 = data.buttons[7]
        old_catapult_2 = data.buttons[6]

        # If more than 2 buttons pressed at same time then
        # priority: zero -> home -> catapult position 1 -> catapult position 2
        if circle_button:
            self.zero()
        elif square_button:
            self.home()
        elif triangle_button:
            self.catapult_1()
        elif cross_button:
            self.catapult_2()
        elif old_catapult_1:
            self.old_catapult_1()
        elif old_catapult_2:
            self.old_catapult_2()

        if vel_acc_scale_inc:
            self.increase_vel_acc_scaling()
        elif vel_acc_scale_dec:
            self.decrease_vel_acc_scaling()

        if catapult_height_inc:
            self.increase_catapult_height()
        elif catapult_height_dec:
            self.decrease_catapult_height()


def main():
    interface = MoveGroupInterface()
    rospy.spin()


if __name__ == '__main__':
    main()
