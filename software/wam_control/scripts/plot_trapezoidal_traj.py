import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
import actionlib
from sensor_msgs.msg import JointState
import time
from geometry_msgs.msg import Point
from wam_control.srv import *
import matplotlib.pyplot as plt
import numpy as np

catapult_init = {
    "wam/base_yaw_joint": -1.13,
    "wam/shoulder_pitch_joint": -1.57,
    "wam/shoulder_yaw_joint": 1.57,
    "wam/elbow_pitch_joint": 0.9,
    "wam/wrist_yaw_joint": -1.571,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

catapult_mid = {
    "wam/base_yaw_joint": 0.0,
    "wam/shoulder_pitch_joint": -0.97,
    "wam/shoulder_yaw_joint": 1.57,
    "wam/elbow_pitch_joint": 0.0,
    "wam/wrist_yaw_joint": -1.571,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

catapult_end = {
    "wam/base_yaw_joint": 1.13,
    "wam/shoulder_pitch_joint": 0,
    "wam/shoulder_yaw_joint": 1.57,
    "wam/elbow_pitch_joint": -0.9,
    "wam/wrist_yaw_joint": -1.571,
    "wam/wrist_pitch_joint": 0,
    "wam/palm_yaw_joint": 0
}

client = None
time_to_hit = None

def plot_data(resp):
    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

    plt.rc('font', **font)

    trajectory = resp.trajectory
    base_yaw_pos = []
    base_yaw_vel = []
    shoulder_pitch_pos = []
    shoulder_pitch_vel = []
    shoulder_yaw_pos = []
    shoulder_yaw_vel = []
    elbow_pitch_pos = []
    elbow_pitch_vel = []
    wrist_yaw_pos = []
    wrist_yaw_vel = []
    times = []
    for i in range(len(trajectory.points)):
        base_yaw_pos.append(trajectory.points[i].positions[0])
        base_yaw_vel.append(trajectory.points[i].velocities[0])
        shoulder_pitch_pos.append(trajectory.points[i].positions[1])
        shoulder_pitch_vel.append(trajectory.points[i].velocities[1])
        shoulder_yaw_pos.append(trajectory.points[i].positions[2])
        shoulder_yaw_vel.append(trajectory.points[i].velocities[2])
        elbow_pitch_pos.append(trajectory.points[i].positions[3])
        elbow_pitch_vel.append(trajectory.points[i].velocities[3])
        wrist_yaw_pos.append(trajectory.points[i].positions[4])
        wrist_yaw_vel.append(trajectory.points[i].velocities[4])
        times.append(trajectory.points[i].time_from_start.to_sec())
    # fig, axs = plt.subplots(3, 2)
    fig, axs = plt.subplots(1, 1)
    #Plot outcomes
    joint = 0
    # axs[0, 0].plot(times, base_yaw_pos, label='Positions')
    # axs[0, 0].plot(times, base_yaw_vel, label='Velocities')
    # axs[0, 0].set_title('Base Yaw Joint')
    # axs[0, 0].legend(loc='best')
    # axs[0, 0].grid()
    # axs[0, 1].plot(times, shoulder_pitch_pos, label='Positions')
    # axs[0, 1].plot(times, shoulder_pitch_vel, label='Velocities')
    # axs[0, 1].set_title('Shoulder Pitch Joint')
    # axs[0, 1].legend(loc='best')
    # axs[0, 1].grid()
    # axs[1, 0].plot(times, shoulder_yaw_pos, label='Positions')
    # axs[1, 0].plot(times, shoulder_yaw_vel, label='Velocities')
    # axs[1, 0].set_title('Shoulder Yaw Joint')
    # axs[1, 0].legend(loc='best')
    # axs[1, 0].grid()
    # axs[1, 1].plot(times, elbow_pitch_pos, label='Positions')
    # axs[1, 1].plot(times, elbow_pitch_vel, label='Velocities')
    # axs[1, 1].set_title('Elbow Pitch Joint')
    # axs[1, 1].legend(loc='best')
    # axs[1, 1].grid()
    # axs[2, 0].plot(times, wrist_yaw_pos, label='Positions')
    # axs[2, 0].plot(times, wrist_yaw_vel, label='Velocities')
    # axs[2, 0].set_title('Wrist Yaw Joint')
    # axs[2, 0].legend(loc='best')
    # axs[2, 0].grid()

    base_yaw_vel = [abs(val) for val in base_yaw_vel]
    shoulder_pitch_vel = [abs(val) for val in shoulder_pitch_vel]
    elbow_pitch_vel = [abs(val) for val in elbow_pitch_vel]

    trapezoidal_trajs = [times, base_yaw_vel, shoulder_pitch_vel, elbow_pitch_vel]
    trapezoidal_trajs_npy = np.array(trapezoidal_trajs).T

    print(trapezoidal_trajs_npy.shape)

    # np.save('trapezoidal_trajs_npy.npy', trapezoidal_trajs_npy)

    # axs.plot(times, [abs(val) for val in base_yaw_vel], label='Base Yaw Joint')
    # axs.plot(times, [abs(val) for val in shoulder_pitch_vel], label='Shoulder Pitch Joint')
    # axs.plot(times, [abs(val) for val in elbow_pitch_vel], label='Elbow Pitch Joint')
    axs.plot(trapezoidal_trajs_npy[:, 0], trapezoidal_trajs_npy[:, 1], label='Base Yaw Joint')
    axs.plot(trapezoidal_trajs_npy[:, 0], trapezoidal_trajs_npy[:, 2], label='Shoulder Pitch Joint')
    axs.plot(trapezoidal_trajs_npy[:, 0], trapezoidal_trajs_npy[:, 3], label='Elbow Pitch Joint')
    plt.axvline(x = 0, linestyle='dashed', color = 'k')
    plt.axvline(x = 0.29, linestyle='dashed', color = 'k')
    plt.axvline(x = resp.mid_arrival, linestyle='dashed', color = 'r', label = 'Planned Contact Point')
    plt.axvline(x = 0.86, linestyle='dashed', color = 'k')
    plt.axvline(x = resp.end_arrival, linestyle='dashed', color = 'k')
    axs.grid(which='major', color='#DDDDDD', linewidth=1.2)
    axs.grid(which='minor', color='#EEEEEE', linestyle=':', linewidth=0.9)
    axs.minorticks_on()
    # axs.grid()
    axs.legend(loc='upper right')
    axs.set_title('Commanded Velocity Profiles of Individual Joints during Swing')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Absolute Angular Speed (rad/s)')

    plt.show()



if __name__ == "__main__":
    print("here")
    
    rospy.init_node('aaaaa', anonymous=True)
    
    print("here2")

    # trajectory profiler
    print("Waiting for trajectory profiler service")
    rospy.wait_for_service('/trajectory_profiler')
    print("Connected to trajectory profiler")

    traj_generator = rospy.ServiceProxy('/trajectory_profiler', TrajectoryProfiler, persistent=True)

    # Go to start 

    service_request = TrajectoryProfilerRequest()

    # curr_joint_position = get_joint_states()
    # curr_joint_position = dict([(name, curr_joint_position[name]) for name in catapult_init.keys()])
    service_request.beg_positions = list(catapult_init.values())
    service_request.mid_positions = list(catapult_mid.values())
    service_request.end_positions = list(catapult_end.values())
    # service_request.get_to_mid_at_time = rospy.get_rostime()
    resp = traj_generator(service_request)
    # print(resp)
    print("--- Time to get to Midpoint %s seconds ---" % resp.mid_arrival)
    print("--- Time of Complete Swing %s seconds ---" % resp.end_arrival)
    plot_data(resp)

    # time.sleep(2)
    # for _ in range(1):
    #     try:
    #         start_time = time.time()
    #         curr_joint_position = get_joint_states()
    #         curr_joint_position = dict([(name, curr_joint_position[name]) for name in catapult_init.keys()])
    #         service_request.start_positions = list(curr_joint_position.values())
    #         service_request.mid_positions = list(catapult_mid.values())
    #         service_request.goal_positions = list(catapult_end.values())
    #         service_request.get_to_mid_at_time = rospy.get_rostime()
    #         service_request.hit_point = Point(0, 0, 0.837 + 0.3)
    #         resp = traj_generator(service_request)
    #         time_to_run_service = time.time()
    #         print("--- Time to run service %s seconds ---" % (time_to_run_service - start_time))
    #         print("--- Time to get to Midpoint %s seconds ---" % resp.time_to_mid)
    #         print("--- Time of Complete Swing %s seconds ---" % resp.time_of_swing)
    #         plot_data(resp)
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     time.sleep(2)

    #     try:
    #         start_time = time.time()
    #         curr_joint_position = get_joint_states()
    #         curr_joint_position = dict([(name, curr_joint_position[name]) for name in catapult_init.keys()])
    #         service_request.start_positions = list(curr_joint_position.values())
    #         service_request.mid_positions = list(catapult_mid.values())
    #         service_request.goal_positions = list(catapult_init.values())
    #         service_request.get_to_mid_at_time = rospy.get_rostime()
    #         service_request.hit_point = Point(0, 0, 0)
    #         resp = traj_generator(service_request)
    #         time_to_run_service = time.time()
    #         print("--- Time to Run Service %s seconds ---" % (time_to_run_service - start_time))
    #         print("--- Time to get to Midpoint %s seconds ---" % resp.time_to_mid)
    #         print("--- Time of Complete Swing %s seconds ---" % resp.time_of_swing)
    #         plot_data(resp)

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)

    #     time.sleep(2)