#!/usr/bin/env python3
"""
Summary: Records transformation between camera and april tag and records
measurements to configuration file
Author: Daniel Martin
"""
import time
import paramiko, psutil
import threading, time
import rospy
from rospy.rostime import Time
import tf
from tf.transformations import euler_from_quaternion
import roslib
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
import yaml
from apriltag_ros.msg import AprilTagDetectionArray
from tf import transformations as t
import json
import rospkg
import ruamel.yaml
import pprint
from ruamel.yaml.scalarstring import (DoubleQuotedScalarString as dq,
                                      SingleQuotedScalarString as sq)
from os.path import exists


sample_time = 10

camera_map = dict()
samples = dict()
times = dict()

# Map ids to bundle [FIXED]
ID_MAP = {
    0: "bundle_1",
    10: "bundle_2",
    21: "bundle_3",
    30: "bundle_4",
}

def getBundle(ids):
    for id, bundle_name in ID_MAP.items():
        if id in ids:
            return bundle_name
    return -1


def apriltag_callback(msg, args):
    camera_name, bundle_name = args

    # Find sample from bundle
    for detection in msg.detections:
        if getBundle(detection.id) == bundle_name or ("opp_" + str(getBundle(detection.id))) == bundle_name:
            samples[camera_name].append(detection.pose.pose.pose)
            times[camera_name] = rospy.Time.now()

output_path = rospkg.RosPack().get_path('ball_calibration') + "/config/camera_calibration_auto.yaml"
yaml = ruamel.yaml.YAML()

def main():
    rospy.init_node('calibration_node')
    global times, samples, camera_map

    # Get "camera_X" : "bundle_Y" mapping from param server
    for i in range(6):
        if rospy.has_param(f"/zed_calibration/camera_{i+1}"):
            bundle_name = rospy.get_param(f"/zed_calibration/camera_{i+1}")
            if str(bundle_name) != "-1":
                camera_map[f"camera_{i+1}"] = bundle_name

    subs = dict()
    for camera_name, bundle_name in camera_map.items():
        samples[camera_name] = []
        times[camera_name] = rospy.Time.now()
        subs[camera_name] = [rospy.Subscriber(f"{camera_name}/tag_detections",
                                     AprilTagDetectionArray, apriltag_callback,
                                    (camera_name, bundle_name), queue_size=1)]

    # Read file
    camera_pose = dict()
    if exists(output_path):
        with open(output_path, 'r') as f:
            camera_pose = yaml.load(f)
            
    if camera_pose == None:
        camera_pose = dict()

    print("[INFO] Starting Calibration")

    rospy.sleep(rospy.Duration(sample_time))

    listener = tf.TransformListener()

    print("[INFO] Processing Samples")

    for camera_name, samples_list in samples.items():

        process_samples = []
        for sample in samples_list:
            # Pose to tranform [left_camera_optical -> bundle to camera_center -> bundle]
            try:
                listener.waitForTransform(f'/{camera_name}_camera_center', f'/{camera_name}_left_camera_optical_frame',
                                          rospy.Time.now(), rospy.Duration(5))
                ps = listener.transformPose(f'/{camera_name}_camera_center',
                    PoseStamped(header=Header(frame_id=f'/{camera_name}_left_camera_optical_frame'), pose=sample))
            except (tf.LookupException):
                print(f"[Error] Couldn't find /{camera_name}_camera_center to /{camera_name}_left_camera_optical_frame !")
                break

            trans = [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            rot = [ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]
            transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

            # Inverse transform [bundle -> camera_center]
            inversed_transform = t.inverse_matrix(transform)
            trans = tf.transformations.translation_from_matrix(inversed_transform)
            rot = tf.transformations.euler_from_matrix(inversed_transform)

            process_samples.append(list(trans) + list(rot))

        camera_pose[camera_name] = {}
        camera_pose[camera_name]["parent_tf"] = "box_" + camera_map[camera_name]
        if process_samples:
            process_samples = np.asarray(process_samples)
            avg = [str(round(float(v), 5)) for v in np.mean(process_samples, axis=0)]
            std = [str(round(float(v), 5)) for v in np.std(process_samples, axis=0)]

            camera_pose[camera_name]["xyz"] = dq(" ".join(avg[:3]))
            camera_pose[camera_name]["rpy"] = dq(" ".join(avg[3:]))
            camera_pose[camera_name]["xyz_std"] = dq(" ".join(std[:3]))
            camera_pose[camera_name]["rpy_std"] = dq(" ".join(std[3:]))
            camera_pose[camera_name]["samples"] = len(process_samples)

        else:
            camera_pose[camera_name]["xyz"] = dq("0 0 0")
            camera_pose[camera_name]["rpy"] = dq("0 0 0")
            camera_pose[camera_name]["xyz_std"] = dq("0 0 0")
            camera_pose[camera_name]["rpy_std"] = dq("0 0 0")
            camera_pose[camera_name]["samples"] = 0

    print("[INFO] Finished Calibration")

    print("Camera Poses:")
    pprint.pprint(camera_pose)

    # Write to file
    with open(output_path, 'w') as f:
        yaml.dump(camera_pose, f)

if __name__ == "__main__":
    main()
