#!/usr/bin/env python3
"""
Summary: Used to evaluate how good the prediction of the EKF compares to 
the ball position hisotry
Author: Daniel Martin
"""
from cProfile import label
from matplotlib import axes
import rosbag
import rospy
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
import matplotlib.lines as mlines
import rospkg


import warnings
warnings.simplefilter('ignore', np.RankWarning)

class EvalTrajectory:
    def __init__(self, path):
        self.MAX_TRAJ_TIME = 3
        self.BOUNCE_START_HEIGHT = 0.3
        self.BOUNCE_COMMIT_HEIGHT = 0.1

        self.selected_trajectory = 0
        self.selected_axis = 2
        self.selected_rollout = 0

        # Read bag
        self.read_bag(path)
        self.process_path()


    def read_bag(self, path):
        bag = rosbag.Bag(path)

        start_times = []
        measurements = []   # [[x, y, z, time], ...]
        measurements_1 = []   # [[x, y, z, time], ...]
        measurements_2 = []   # [[x, y, z, time], ...]
        measurements_3 = []   # [[x, y, z, time], ...]
        measurements_4 = []   # [[x, y, z, time], ...]
        measurements_5 = []   # [[x, y, z, time], ...]
        measurements_6 = []   # [[x, y, z, time], ...]
        predictions = []    # [[x, y, z, vx, vy, vz, time], ...]
        rollouts = []       # [[[x, y, z, time], ...,], ...]
        hitpoints = []

        for topic, msg, t in bag.read_messages():
            # Find start and end time
            if topic == "/ball/event":
                print("Hereeeeeeeeee")
                print(msg)
                if msg.data == "Launch" or msg.data == "Hit":
                # if msg.data == "Launch" or msg.data == "Hit" or msg.data == "Reset":
                    print(msg)
                    start_times.append(t.to_sec())

            if topic == "/ball/set_pose":
                print("Set pose")
                start_times.append(t.to_sec())

            if start_times:
                if "_transformed" in topic:
                # if topic in ["/ball_1_pose_transformed", "/ball_2_pose_transformed", "/ball_3_pose_transformed", "/ball_4_pose_transformed", "/ball_5_pose_transformed"]:
                    if int(topic[6]) in [1, 2, 4, 5, 6]: #[1, 2, 4, 5, 6]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        # camera_id = float(topic[5]) / 5
                        measurements.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [1]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_1.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [2]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_2.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [3]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_3.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [4]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_4.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [5]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_5.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    if int(topic[6]) in [6]:
                        p = msg.pose.position
                        camera_id = float(topic[6]) / 6
                        measurements_6.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
                    
                elif topic == "/ball_odometry":
                    p = msg.pose.pose.position
                    v = msg.twist.twist.linear
                    predictions.append([p.x, p.y, p.z, v.x, v.y, v.z, msg.header.stamp.to_sec()])
                elif topic == "/ball/rollout/path":
                    rollout = []
                    for pose_msg in msg.poses:
                        p = pose_msg.pose.position
                        stamp = pose_msg.header.stamp
                        rollout.append([p.x, p.y, p.z, stamp.to_sec()])
                    rollouts.append(rollout)
                if topic == "/hit_ball_point":
                    p = msg.point
                    hitpoints.append([p.x, p.y, p.z, camera_id, msg.header.stamp.to_sec()])
        
        # print(hitpoints)
        # print(len(hitpoints))
        hitpoints = sorted(hitpoints, key=lambda p: p[-1])
        self.orig_hitpoints = hitpoints

        self.start_times = start_times
        print(start_times)
        predictions = sorted(predictions, key=lambda p: p[-1])
        rollouts = sorted(rollouts, key=lambda p: p[0][-1])
        # self.orig_measurements, self.orig_predictions, self.orig_rollouts = \
        #     np.array(measurements), np.array(predictions), np.array(rollouts)

        self.orig_measurements, self.orig_measurements_1, self.orig_measurements_2, self.orig_measurements_3, self.orig_measurements_4, self.orig_measurements_5, self.orig_measurements_6, self.orig_predictions, self.orig_rollouts = \
            np.array(measurements), np.array(measurements_1), np.array(measurements_2), np.array(measurements_3), np.array(measurements_4), np.array(measurements_5), np.array(measurements_6), np.array(predictions), np.array(rollouts)


        bag.close()

        # print(self.orig_measurements.shape)
        # print(self.orig_predictions.shape)
        # print(self.orig_rollouts.shape)

    # Trim path
    def align_path(self, path, start_t, end_t, numpy=True, rollout=False):
        # Align 3D rollout
        if rollout:
            rollouts = path
            new_rollouts = []
            for rollout in rollouts:
                new_rollout = self.align_path(rollout, start_t, end_t, numpy=True)
                if len(new_rollout) > 0:
                    new_rollouts.append(new_rollout)
            return new_rollouts

        # Align 2D path
        trim_path = []
        for point in path:
            *data, t = point
            if start_t <= t < (start_t + end_t):
                trim_path.append([*data, t - start_t])
        return np.array(trim_path) if numpy else trim_path


    def get_bounce_times(self, path):
        bounce_times = []
        state = "up"
        min_z = np.inf
        min_time = None

        for point in path:
            x, y, z, id, t = point

            if state == "up" and z < self.BOUNCE_START_HEIGHT:
                state = "bouncing"

            if state == "bouncing" and z > self.BOUNCE_COMMIT_HEIGHT:
                state = "up"
                if min_z < .1:
                    bounce_times.append(min_time)
                min_z = np.inf

            if state == "bouncing":
                if z < min_z:
                    min_z = z
                    min_time = t

        return bounce_times


    def isolate_ballistic(self, start_t, end_t, path):
        start_idx, stop_idx = 0, len(path)
        for i, t in enumerate(path[:, -1]):
            if t <= start_t:
                start_idx = i
            if t >= end_t:
                stop_idx = i
                break
        return path[start_idx:stop_idx]


    def fit_trajectory(self, measurements, bounce_times):
        fits = None
        for i in range(len(bounce_times)+1):
            if i == 0:
                s, e = 0, bounce_times[0]
            elif i == len(bounce_times):
                s, e = bounce_times[i-1], np.inf
            else:
                s, e = bounce_times[i-1], bounce_times[i]

            trim_measurements = self.isolate_ballistic(s, e, measurements)
            time = trim_measurements[:, -1]

            if len(trim_measurements) == 0:
                continue

            fit = np.zeros((len(time), 7))
            for axis in range(3):
                samples = trim_measurements[:, axis]

                poly = np.poly1d(np.polyfit(time, samples, 2))
                poly_deriv = np.polyder(poly)

                fit[:, axis] = np.array([poly(t) for t in time])
                fit[:, axis+3] = np.array([poly_deriv(t) for t in time])
            fit[:, -1] = time

            fits = np.vstack((fits, fit)) if (fits is not None) else fit
        return fits


    def find_nearest(self, array, value):
        idx = (np.abs(array - value)).argmin()
        return idx, array[idx]


    def create_trajectory(self, selected_traj):
        print(selected_traj)
        print(len(self.orig_predictions))
        start_time = self.start_times[selected_traj]
        next_start = self.start_times[selected_traj +1] if selected_traj != len(self.start_times)-1 else np.inf
        end_time = min(self.MAX_TRAJ_TIME, next_start - start_time)

        # if next_start - start_time < 2:
        #     self.selected_trajectory += 1
        #     self.create_trajectory(self.selected_trajectory)
        #     return

        # Shift and trim
        self.measurements = self.align_path(self.orig_measurements, start_time, end_time)

        self.measurements_1 = self.align_path(self.orig_measurements_1, start_time, end_time)
        self.measurements_2 = self.align_path(self.orig_measurements_2, start_time, end_time)
        self.measurements_3 = self.align_path(self.orig_measurements_3, start_time, end_time)
        self.measurements_4 = self.align_path(self.orig_measurements_4, start_time, end_time)
        self.measurements_5 = self.align_path(self.orig_measurements_5, start_time, end_time)
        self.measurements_6 = self.align_path(self.orig_measurements_6, start_time, end_time)

        self.predictions = self.align_path(self.orig_predictions, start_time, end_time)
        self.rollouts = self.align_path(self.orig_rollouts, start_time, end_time, rollout=True)

        self.hitpoints = self.align_path(self.orig_hitpoints, start_time, end_time)

        # if self.hitpoints.size == 0 or self.hitpoints.size == 1:
        #     self.selected_trajectory += 1
        #     self.create_trajectory(self.selected_trajectory)
        #     return

        # if self.predictions.size == 0:
        if self.measurements.size < 50:
            self.selected_trajectory += 1
            self.create_trajectory(self.selected_trajectory)
            return

        if self.predictions.size < 50:
            self.selected_trajectory += 1
            self.create_trajectory(self.selected_trajectory)
            return

        self.bounce_times = self.get_bounce_times(self.measurements)
        self.fits = self.predictions 
        # self.fits = self.fit_trajectory(self.measurements, self.bounce_times) # Sometimes errors out
        # self.fits = self.fit_trajectory(self.predictions, self.bounce_times) # Sometimes errors out


    def process_path(self):
        self.create_trajectory(self.selected_trajectory)

        # Plot
        fig, ax = plt.subplots()

        # self.scatter = ax.scatter(self.measurements[:, -1], self.measurements[:, self.selected_axis], s=50, c=self.measurements[:, 3], cmap="cool", label="Camera Estimates")
        self.scatter_1 = ax.scatter(self.measurements_1[:, -1], self.measurements_1[:, self.selected_axis], s=50, c='g', cmap="cool", label="Camera 1 Estimate")
        self.scatter_2 = ax.scatter(self.measurements_2[:, -1], self.measurements_2[:, self.selected_axis], s=50, c='r', cmap="cool", label="Camera 2 Estimate")
        # self.scatter_3 = ax.scatter(self.measurements_3[:, -1], self.measurements_3[:, self.selected_axis], s=50, c='c', cmap="cool", label="Camera 3 Estimate")
        self.scatter_4 = ax.scatter(self.measurements_4[:, -1], self.measurements_4[:, self.selected_axis], s=50, c='m', cmap="cool", label="Camera 4 Estimate")
        self.scatter_5 = ax.scatter(self.measurements_5[:, -1], self.measurements_5[:, self.selected_axis], s=50, c='y', cmap="cool", label="Camera 5 Estimate")
        self.scatter_6 = ax.scatter(self.measurements_6[:, -1], self.measurements_6[:, self.selected_axis], s=50, c='k', cmap="cool", label="Camera 6 Estimate")

        fit_plot,  = ax.plot(self.fits[:, -1], self.fits[:, self.selected_axis],  'g-', label="Fit")
        pred_plot, = ax.plot(self.predictions[:, -1], self.predictions[:, self.selected_axis], 'bo-', label="Current EKF Odometery Estimate")
        roll_plot, = ax.plot(self.rollouts[self.selected_rollout][:, -1],
                             self.rollouts[self.selected_rollout][:, self.selected_axis],  'r--', label="Rollout")
        
        # try:
        #     a = self.hitpoints[:, -1]
        #     b = self.hitpoints[:, self.selected_axis]
        #     hitpoint_scatter, = ax.plot( a, b, 'k*', label="Hitpoint")
        # except:
        #     pass
        # hitpoint_scatter, = ax.plot(self.hitpoints[:, -1], self.hitpoints[:, self.selected_axis], 'k*', label="Hitpoint")

        ax.set_ylabel('Position (meters)')
        ax.set_xlabel('Time (seconds)')
        ax.grid()
        leg = ax.legend()

        # Slider
        plt.subplots_adjust(bottom=0.3)
        sax = plt.axes([0.20, 0.15, 0.65, 0.03])
        roll_slider = plt.Slider(sax, valmin=0, valmax=1, valinit=self.selected_rollout, label="Rollout %")
        
        sax = plt.axes([0.20, 0.1, 0.65, 0.03])
        traj_slider = plt.Slider(sax, valmin=0, valmax=len(self.start_times)-1, valinit=self.selected_trajectory, label="Trajectory #", valstep=1)
        
        sax = plt.axes([0.20, 0.05, 0.65, 0.03])
        axis_slider = plt.Slider(sax, valmin=0, valmax=2, valinit=2, label="X | Y | Z Axis", valstep=1)
    
        def update_rollout(rollout_per):
            self.selected_rollout = rollout_num = int(rollout_per * (len(self.rollouts)-1))
            roll_plot.set_ydata(self.rollouts[rollout_num][:, self.selected_axis])
            roll_plot.set_xdata(self.rollouts[rollout_num][:, -1])
            # print("hereeeeeeeeee")
            # print(self.rollouts[rollout_num][:, -1])
            # print("thereeeeeeeeee")
            # print(fit_plot.get_xdata())
            fig.canvas.draw()
        roll_slider.on_changed(update_rollout)

        def update_trajectory(selected_traj):
            self.selected_trajectory = selected_traj = int(selected_traj)
            rollout_per = self.selected_rollout/(len(self.rollouts)-1)
            self.create_trajectory(selected_traj)
            update_rollout(rollout_per)
            update_axis(self.selected_axis)
        traj_slider.on_changed(update_trajectory)

        def update_axis(axis):
            self.selected_axis = axis = int(axis)

            # self.scatter.remove()
            # try:
            # self.scatter = ax.scatter(self.measurements[:, -1], self.measurements[:, self.selected_axis], s=50, c=self.measurements[:, 3], cmap="cool", label="Camera Estimates")
            # except:
            #     ff
            self.scatter_1.remove()
            self.scatter_2.remove()
            # self.scatter_3.remove()
            self.scatter_4.remove()
            self.scatter_5.remove()
            self.scatter_6.remove()
            self.scatter_1 = ax.scatter(self.measurements_1[:, -1], self.measurements_1[:, self.selected_axis], s=50, c='g', cmap="cool", label="Camera 1 Estimate")
            self.scatter_2 = ax.scatter(self.measurements_2[:, -1], self.measurements_2[:, self.selected_axis], s=50, c='r', cmap="cool", label="Camera 2 Estimate")
            # self.scatter_3 = ax.scatter(self.measurements_3[:, -1], self.measurements_3[:, self.selected_axis], s=50, c='c', cmap="cool", label="Camera 3 Estimate")
            self.scatter_4 = ax.scatter(self.measurements_4[:, -1], self.measurements_4[:, self.selected_axis], s=50, c='m', cmap="cool", label="Camera 4 Estimate")
            self.scatter_5 = ax.scatter(self.measurements_5[:, -1], self.measurements_5[:, self.selected_axis], s=50, c='y', cmap="cool", label="Camera 5 Estimate")
            self.scatter_6 = ax.scatter(self.measurements_6[:, -1], self.measurements_6[:, self.selected_axis], s=50, c='k', cmap="cool", label="Camera 6 Estimate")

            pred_plot.set_ydata(self.predictions[:, axis])
            pred_plot.set_xdata(self.predictions[:, -1])
            roll_plot.set_ydata(self.rollouts[self.selected_rollout][:, axis])
            roll_plot.set_xdata(self.rollouts[self.selected_rollout][:, -1])
            fit_plot.set_ydata(self.fits[:, axis])
            fit_plot.set_xdata(self.fits[:, -1])
            # hitpoint_scatter.set_ydata(self.hitpoints[:, axis])
            # hitpoint_scatter.set_xdata(self.hitpoints[:, -1])

            # try:
            #     a = self.hitpoints[:, -1]
            #     b = self.hitpoints[:, self.selected_axis]
            #     hitpoint_scatter, = ax.plot( a, b, 'k*', label="Hitpoint")
            # except:
            #     pass

            min_v = min(np.min(self.measurements[:, axis]), np.min(self.predictions[:, axis]))
            max_v  = max(np.max(self.measurements[:, axis]), np.max(self.predictions[:, axis]))
            ax.set_ylim([min_v, max_v])

            min_v = min(np.min(self.measurements[:, -1]), np.min(self.predictions[:, -1]))
            max_v  = max(np.max(self.measurements[:, -1]), np.max(self.predictions[:, -1]))
            ax.set_xlim([min_v, max_v])

            fig.canvas.draw()
        axis_slider.on_changed(update_axis)

        # Hover
        circle_fit  = plt.Circle((-1, -1), .03, color='green', zorder=999)
        circle_pred = plt.Circle((-1, -1), .03, color='blue',  zorder=997)
        circle_roll = plt.Circle((-1, -1), .03, color='red',   zorder=998)
        ax.add_artist(circle_fit); ax.add_artist(circle_pred); ax.add_artist(circle_roll)

        def on_move(event):
            if event.inaxes:
                axis, rollout_num  = self.selected_axis, self.selected_rollout

                # Fit
                fit_i, fit_t = self.find_nearest(self.fits[:,-1], event.xdata)
                leg.texts[0].set_text(
                    f"Fit:    xy=({fit_t:.2f},{self.fits[fit_i, axis]:.2f}) v=({self.fits[fit_i, axis+3]:.2f})")
                circle_fit.center = fit_t, self.fits[fit_i, axis]

                # Pred
                pred_i, pred_t = self.find_nearest(self.predictions[:,-1], event.xdata)
                leg.texts[1].set_text(
                    f"Pred: xy=({pred_t:.2f},{self.predictions[pred_i, axis]:.2f}) v=({self.predictions[pred_i, axis+3]:.2f}) " +
                    f"dV=({self.fits[fit_i, axis+3] - self.predictions[pred_i, axis+3]:.2f})")
                circle_pred.center = pred_t, self.predictions[pred_i, axis]

                # Rollout
                roll_i, roll_t = self.find_nearest(self.rollouts[rollout_num][:, -1], event.xdata)
                roll_deriv = np.gradient(self.rollouts[rollout_num][:, axis], self.rollouts[rollout_num][:, -1])
                leg.texts[2].set_text(
                    f"Roll:  xy=({roll_t:.2f},{self.rollouts[rollout_num][roll_i, axis]:.2f}) v=({roll_deriv[roll_i]:.2f}) " +
                    f"dV=({self.fits[fit_i, axis+3] - roll_deriv[roll_i]:.2f})")
                circle_roll.center = roll_t, self.rollouts[rollout_num][roll_i, axis]

                fig.canvas.draw()
        binding_id = plt.connect('motion_notify_event', on_move)

        plt.show()


# path = rospkg.RosPack().get_path('ball_calibration') + '/bags/vision_2022-07-01-13-15-29.bag'
path = rospkg.RosPack().get_path('ball_calibration') + '/bags/vision_2022-09-20-06-54-24.bag'
# path = '/home/core-robotics/court_bags/new_ball_launcher/new_cam_pos_courts_2022-07-15-13-55-13.bag'
EvalTrajectory(path)


