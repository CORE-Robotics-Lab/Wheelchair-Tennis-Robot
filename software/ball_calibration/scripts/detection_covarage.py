#!/usr/bin/env python3
"""
Summary: Graphs how often we see the ball from each camera across time
Author: Daniel Martin
"""
from tracemalloc import start
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import collections


bag = rosbag.Bag('/home/dani/Downloads/court_feb_16_full.bag')

colors_dict = {
    "/ball_1_pose": 'red',
    "/ball_2_pose": 'orange',
    "/ball_3_pose": 'yellow',
    "/ball_4_pose": 'green',
    "/ball_5_pose": 'blue',
    "/ball_6_pose": 'purple',
}

times = []
times_dict = collections.defaultdict(list)

start_time = None
end_time = None
for topic, msg, t in bag.read_messages():
    # Find start and end time
    if not start_time or t < start_time:
        start_time = t
    if not end_time or t > end_time:
        end_time = t

    if "ball" in topic:
        times.append((t - start_time).to_sec())
        times_dict[topic].append((t - start_time).to_sec())

bag.close()

bins_per_sec = 30 # Hz
num_bins = int((end_time - start_time).to_sec() * bins_per_sec)
bins_seq = np.linspace(0, int((end_time - start_time).to_sec()), num_bins)

times_2D = [times_dict[key] for key in colors_dict.keys()]
colors = list(colors_dict.values())
labels = list(colors_dict.keys())


individual_cameras = True
if individual_cameras:
    n, bins, patches = plt.hist(times_2D, bins_seq, alpha=0.75, color=colors, label=labels, stacked=True)
else:
    n, bins, patches = plt.hist(times, bins_seq, alpha=0.75)

    avg_detections = np.mean(n)    # Avg num of cameras detecting ball per bin
    avg_detected = np.mean(n >= 1) # Percent that at least 1 camera is detecting ball per bin

    print(f"avg_detections: {avg_detections}")
    print(f"avg_detected: {avg_detected}")


plt.title('Ball Coverage')
plt.legend()
plt.show()
