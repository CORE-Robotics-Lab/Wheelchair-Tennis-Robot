#!/usr/bin/env python
import os
import sys
import inspect
import numpy as np
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
import matplotlib.pyplot as plt

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

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        
        # Load joint limits
        path = rospkg.RosPack().get_path('wam_moveit') + "/config/joint_limits.yaml"
        with open(path, 'r') as file:
            joint_limits = yaml.safe_load(file)["joint_limits"]

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        
        # print("============ Printing current end effector pose")
        # print(move_group.get_current_pose())

        # self.joint_pos_catapult = np.load('joint_positions_catapult.npy')
        self.joint_pos_catapult = np.load('joint_positions_pingpong.npy')
        # print(self.joint_pos_catapult.shape)

        self.end_effector_pos = []

    def go_to_start(self):
        joint_goals = self.joint_pos_catapult[0][:]
        result = self.move_group.go(joint_goals, wait=True)
        self.move_group.stop()

        end_effector_pose = self.move_group.get_current_pose()
        self.end_effector_pos.append([end_effector_pose.pose.position.x, end_effector_pose.pose.position.y, end_effector_pose.pose.position.z])
        # print(end_effector_pose)
        

    def execute_swing(self):
        self.times_pingpong_real = np.load('timestamps_pingpong.npy')
        time_index_to_delete = []
        for i in range(1, len(self.joint_pos_catapult)):
            try:
                joint_goals = self.joint_pos_catapult[i][:].tolist()
                result = self.move_group.go(joint_goals, wait=True)
                end_effector_pose = self.move_group.get_current_pose()
                self.end_effector_pos.append([end_effector_pose.pose.position.x, end_effector_pose.pose.position.y, end_effector_pose.pose.position.z])
            except:
                print(i)
                time_index_to_delete.append(i)
        print("Deleting time indices")
        self.times_pingpong_real = np.delete(self.times_pingpong_real, time_index_to_delete)

        print("Saving Time indices")
        np.save('timestamps_pingpong_real', self.times_pingpong_real)

    def plot_end_effector_pose(self):
        fig, axs = plt.subplots(1, 1)

        axs.plot([val[0] for val in self.end_effector_pos], label='x')
        axs.plot([val[1] for val in self.end_effector_pos], label='y')
        axs.plot([val[2] for val in self.end_effector_pos], label='z')

        axs.grid(which='major', color='#DDDDDD', linewidth=1.2)
        axs.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.9)
        axs.minorticks_on()
        # axs.grid()
        axs.legend(loc='best')
        axs.set_title('x,y,z w.r.t base footprint of racket hitpoint')
        plt.xlabel('Steps')
        plt.ylabel('Position (meters)')

        plt.show()

        end_effector_pos = np.array(self.end_effector_pos)

        print(end_effector_pos.shape)
        np.save('end_effector_xyz_pingpong_real', end_effector_pos)
    
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    from math import factorial
    
    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')


def get_velocity(times, positons):
    vel_x = [0]
    vel_y = [0]
    vel_z = [0]

    for i in range(1, len(times)):
        x_vel_i = (positons[i, 0] - positons[i-1, 0])/(times[i] - times[i-1])
        y_vel_i = (positons[i, 1] - positons[i-1, 1])/(times[i] - times[i-1])
        z_vel_i = (positons[i, 2] - positons[i-1, 2])/(times[i] - times[i-1])

        if abs(x_vel_i) > 15:
            x_vel_i = vel_x[-1]
        if abs(y_vel_i) > 15:
            y_vel_i = vel_y[-1]
        if abs(z_vel_i) > 15:
            z_vel_i = vel_z[-1]

        vel_x.append(x_vel_i)
        vel_y.append(y_vel_i)
        vel_z.append(z_vel_i)

    return vel_x, vel_y, vel_z

def combine_x_y_velocity(x_vel, y_vel):
    xy_vel = []
    for i in range(len(x_vel)):
        xy_vel.append(np.sqrt(np.square(x_vel[i]) + np.square(y_vel[i])))

    return xy_vel

def combine_x_y_z_velocity(x_vel, y_vel, z_vel):
    xyz_vel = []
    for i in range(len(x_vel)):
        xyz_vel.append(np.sqrt(np.square(x_vel[i]) + np.square(y_vel[i]) + np.square(z_vel[i])))

    return xyz_vel

def plot_end_effector_speed():
    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

    plt.rc('font', **font)

    end_effector_xyz_catapult = np.load('end_effector_xyz_catapult.npy')
    # end_effector_xyz_catapult = np.load('end_effector_xyz_pingpong_real.npy')
    print(end_effector_xyz_catapult.shape)

    timestamps = np.load('timestamps_catapult.npy')
    timestamps = np.delete(timestamps, [208, 209])

    # timestamps = np.load('timestamps_pingpong_real.npy')
    timestamps = timestamps - timestamps[0]
    
    print(timestamps.shape)

    vel_x, vel_y, vel_z = get_velocity(timestamps, end_effector_xyz_catapult)

    window = 51
    order = 2
    vel_x_smoothed = savitzky_golay(np.array(vel_x), window, order)
    vel_y_smoothed = savitzky_golay(np.array(vel_y), window, order)
    vel_z_smoothed = savitzky_golay(np.array(vel_z), window, order)

    vel_xy_smoothed = combine_x_y_velocity(vel_x_smoothed, vel_y_smoothed)

    vel_xyz_smoothed = combine_x_y_z_velocity(vel_x_smoothed, vel_y_smoothed, vel_z_smoothed)

    # # vel_xyz_smoothed[-8] = 0
    # vel_xyz_smoothed[-7] = 0
    # vel_xyz_smoothed[-6] = 0
    # # vel_xyz_smoothed[-5] = 0
    # vel_xyz_smoothed[-4] = 0
    # vel_xyz_smoothed[-3] = 0
    # vel_xyz_smoothed[-2] = 0
    # vel_xyz_smoothed[-1] = 0
    

    fig, axs = plt.subplots(1, 1)

    # axs.plot(timestamps, end_effector_xyz_catapult[:, 0], label='x')
    # axs.plot(timestamps, end_effector_xyz_catapult[:, 1], label='y')
    # axs.plot(timestamps, end_effector_xyz_catapult[:, 2], label='z')

    # axs.plot(timestamps, vel_x, label='vel_x')
    # axs.plot(timestamps, vel_y, label='vel_y')
    # axs.plot(timestamps, vel_z, label='vel_z')

    # axs.plot(timestamps, vel_x_smoothed, label='vel_x_smoothed')
    # axs.plot(timestamps, vel_y_smoothed, label='vel_y_smoothed')
    # axs.plot(timestamps, vel_z_smoothed, label='vel_z_smoothed')
    # axs.plot(timestamps, vel_xy_smoothed, label='vel_return_direction')
    axs.plot(timestamps[:], vel_xyz_smoothed[:], label = 'Racket Head Speed')
    # plt.axvline(x = 0.61, linestyle='dashed', color = 'm', label = 'Actual Contact Point')

    plt.axvline(x = 0, linestyle='dashed', color = 'k')
    plt.axvline(x = 0.29, linestyle='dashed', color = 'k')
    plt.axvline(x = 0.57, linestyle='dashed', color = 'r', label = 'Planned Contact Point')
    plt.axvline(x = 0.86, linestyle='dashed', color = 'k')
    plt.axvline(x = 1.15, linestyle='dashed', color = 'k')
    axs.grid(which='major', color='#DDDDDD', linewidth=1.2)
    axs.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.9)

    axs.grid(which='major', color='#DDDDDD', linewidth=1.2)
    axs.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.9)
    axs.minorticks_on()
    # axs.grid()
    axs.legend(loc='upper right')
    axs.set_title('Racket Head Speed during an Actual Swing')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed (m/s)')

    plt.show()

    rackethead_speed = [timestamps[:], vel_xyz_smoothed[:]]
    rackethead_npy = np.array(rackethead_speed).T

    print(rackethead_npy.shape)
    print(rackethead_npy)

    np.save('rackethead_speed.npy', rackethead_npy)



def main():
    # interface = MoveGroupInterface()

    # interface.go_to_start()
    # rospy.sleep(0.1)

    # print("Executing swing")
    # interface.execute_swing()
    # print("swing executed")
    # rospy.sleep(1)

    # print(len(interface.end_effector_pos))

    # interface.plot_end_effector_pose()

    plot_end_effector_speed()

    # rospy.spin()


if __name__ == '__main__':
    main()