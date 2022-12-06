# Software

- [Software](#software)
  - [Vision](#vision)
    - [Hardware](#hardware)
    - [Static IP](#static-ip)
    - [Time Sync](#time-sync)
    - [Setup New Jetson](#setup-new-jetson)
    - [Calibration](#calibration)
    - [Ball Detection](#ball-detection)
    - [Position Tracking](#position-tracking)
  - [Wheelchair](#wheelchair)
    - [Localization](#localization)
    - [Planner](#planner)
    - [Strategizer](#strategizer)
  - [Arm](#arm)
    - [Calibration](#calibration-1)
    - [Controller](#controller)


## Vision

[Calibration Instructions](ball_calibration/README.md) 

### Hardware
Our video system is mainly composed of 6 camera tripod stands with a:

- ZED 2 (Stereo Camera)
- Jetson Nano (Compute Module)
- Wifi Antenna (Wireless Data Transmission)
- Battery Pack (Power Supply)

<p align="center">
    <img src="documents/photos/zed_diagram.png" alt="Court Vision" width="400" />
</p>

positioned around the tennis court to capture ball movements. 

All jetsons should have:
- Ubuntu 18.04
- ROS Melodic
- ZED SDK
- OpenCV 4.5.2+ (with CUDA)
- Catkin workspace with `jetson_master` branch
- Username: jetson-nano-\<ID>
- Password: 1111
- IP: `192.168.1.10<ID>`  (IDs range from 1 to 6)


### Static IP

We also have a 5GHz wifi router (`core-robotics-net-5G`) that allows the stands to be connected to a shared network and transmit data to the wheelchair computer. All jetsons should have a static IP based on their assigned ID like `192.168.1.10<ID>`, so jetson with ID 3 should have IP `192.168.1.103` and should be accessible through SSH at `jetson-nano-3@192.168.1.103`. Note that we are actually assigning the IP address to the long-range wifi antennas and not the jetson internal wifi module. We assign the antenna’s MAC address to a static IP within the router’s admin settings. Please ask the project lead if you need the router password. **Each antenna and jetson should have a tag with an ID that it maps to, so don’t interchange antennas and jetsons.**

The master computer should always have the antenna with tag 0 in order to map it to `192.168.1.100`. This is important for roscore and time sync. 

The jetson should automatically connect to `core-robotics-net-2` on boot if the network avialable. You can find all the jetson connected by using `nmap -sP 192.168.1.* | grep "192.168.1.10"`. If a jetson is not connected, you need to ssh into the jetson (likely through ethernet cable) and use `sudo nmcli d wifi connect core-robotics-net-2_5G password ogmcglab`. 

### Time Sync
In order for all the Jetsons to have sync'd time for pose estimates, we use [chrony](https://answers.ros.org/question/298821/tf-timeout-with-multiple-machines/) and let the master computer be the main NTP server.  

### Setup New Jetson
On a 32Gb SD Card, flash the image with all the software requirements from this [link](https://github.com/Qengineering/Jetson-Nano-image). Now to update the username for the jetsons, we can follow the steps given in this [thread](https://forums.developer.nvidia.com/t/changing-the-default-username-on-jetson-tx2/119693/2) and change the home directory with a renamed copy of the original folder. The password should be changed by logging into the root of the system.

To set up the ROS Networking on the Jetsons, change `~/.bashrc` and add: `export ROS_MASTER_URI=http://192.168.1.100:11311` and `export ROS_HOSTNAME=<Jetson IP>`. Make sure Jetson is loaded with all the required packages.

### Calibration

To get the camera’s relative position within the world, we use [Apriltag](http://wiki.ros.org/apriltag_ros) calibration to get accurate pose estimates. We created a box covered in unique Apriltag bundles so that we can calibrate the cameras with a pre-measured box in a known position in the world. All this functionality is located in the `ball_calibration` package.

Refer to court calibration section for main details. 

### Ball Detection

Our current method of detecting an orange tennis ball is to find the center of the “largest moving orange object” within an image. A simple explanation of the approach involves:

- For left and right images of the stereo camera:
  - Perform background subtraction (_find moving objects_)
  - Perform HSV color thresholding (_find orange objects_)
  - Determine the largest area that is moving and contains orange (_find largest_)
  - Find the center of this area
- Perform stereo depth estimation based on left and right centers position
- Convert to pose estimate with predicted measurement covariance

This pose estimate then is sent to the EKF for processing.

We perform this detection process on the jetsons since transmitting image data is quite expensive. Since the jetsons aren’t very powerful, we had to optimize our detections code by using GPU and incorporating some other complex optimizations.

### Position Tracking

Currently, for fusing all the camera’s position estimates we use a modified version of `robot_localization`’s EKF that accounts for the ball’s ballistic dynamics called `ball_localization`. We also incorporated the ability to rollout the ball trajectory into the future by iteratively calling the EKF’s predicting method. Setting for the EKF can be tuned within this [yaml](ball_calibration/config/ball_ekf_localization.yaml) based for of `robot_localization`. 

We tune the EKF, especially the ball bounce parameters, using a script that evulates the bag position history versus the EKF rollout for a particular time using this [testing script](ball_calibration/scripts/evalute_rollout.py)

## Wheelchair

### Localization

### Planner

### Strategizer

## Arm

### Calibration

### Controller
