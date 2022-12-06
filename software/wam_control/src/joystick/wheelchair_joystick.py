#!/usr/bin/env python
import os
import sys
import rospkg
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import ruamel.yaml as yaml
from wtr_navigation.msg import WheelInfo
import math 

class WheelchairCommander():
    """Move Wheelchair with Joystick"""

    def __init__(self):
        rospy.init_node("wheelchair_move_joystick", anonymous=True)
        
        self.LINEAR_VEL_MAX = 1.5     # Maximum linear velocity [m/s]
        self.ANGULAR_VEL_MAX = 1.5    # Maximum angular velocity [rads/s]
        self.WHEEL_SEPARATION = 0.92  # Wheel separation
        self.WHEEL_RADIUS = 0.3225    # Wheel radius
        
        self.throttle = 0             # throttle input mapped from -1 to 1 to -linear max velocity to +linear max velocity
        self.steering = 0             # steering input mapped from -1 to 1 to -angular max velocity to +angualr max velocity

        self.left_wheel_vel = 0       # calculated velocity for left wheel [rad/s]
        self.right_wheel_vel = 0      # calculated velocity for right wheel [rad/s]
        
        # setting up publisher for publishing commanded wheel velocities
        self.cmd_wheel = rospy.Publisher('/wheel_cmd_vel', WheelInfo, queue_size=1)
        
        # Setting up subscriber for joystick
        rospy.Subscriber("joy", Joy, self.js_callback, queue_size=1)


    def js_callback(self, data):
        self.throttle = data.axes[1] * self.LINEAR_VEL_MAX
        self.steering = data.axes[3] * self.LINEAR_VEL_MAX

        #converts throttle and steering inputs to left and right wheel velocities
        self.left_wheel_vel = (self.throttle + (self.steering * self.WHEEL_SEPARATION * 0.5)) / self.WHEEL_RADIUS
        self.right_wheel_vel = (self.throttle - (self.steering * self.WHEEL_SEPARATION * 0.5)) / self.WHEEL_RADIUS

        # publish the commanded velocities
        cmd_wheel_msg = WheelInfo()
        cmd_wheel_msg.left = self.left_wheel_vel
        cmd_wheel_msg.right = self.right_wheel_vel
        self.cmd_wheel.publish(cmd_wheel_msg)

def main():
    wc_js_interface = WheelchairCommander()
    rospy.spin()


if __name__ == '__main__':
    main()
        



        