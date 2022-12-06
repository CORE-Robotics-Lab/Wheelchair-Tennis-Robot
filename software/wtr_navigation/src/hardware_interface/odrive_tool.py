#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
from math import pi 
import rospy
from wtr_navigation.msg import WheelInfo

loop_rate = 250

def main():
    rospy.init_node("odrive_tool", anonymous=False)

    # Find a connected ODrive
    print("[odrive_tool] Finding an odrive...")
    my_drive = odrive.find_any()
    print("[odrive_tool] Connected to odrive...")

    est_pos_pub = rospy.Publisher("/wheel_est_pos", WheelInfo, queue_size=1)
    est_vel_pub = rospy.Publisher("/wheel_est_vel", WheelInfo, queue_size=1)

    rad_per_rev = (2 * pi)
    motor_to_wheel_ratio = 1 / 20

    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        # Get pos and vel estimate and publish
        est_pos_pub.publish(WheelInfo(left=-my_drive.axis0.encoder.pos_estimate * rad_per_rev * motor_to_wheel_ratio, 
                                      right=my_drive.axis1.encoder.pos_estimate * rad_per_rev * motor_to_wheel_ratio))
        est_vel_pub.publish(WheelInfo(left=-my_drive.axis0.encoder.vel_estimate * rad_per_rev * motor_to_wheel_ratio, 
                                      right=my_drive.axis1.encoder.vel_estimate * rad_per_rev * motor_to_wheel_ratio))

        rate.sleep()

if __name__ == '__main__':
    main()
