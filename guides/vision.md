
# Vision Guide
Guide on how to use and set up vision for WTR project. 

## Overview
### Hardware:
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


### Static IP:

We also have a 5GHz wifi router (`core-robotics-net-2`) that allows the stands to be connected to a shared network and transmit data to the wheelchair computer. All jetsons should have a static IP based on their assigned ID like `192.168.1.10<ID>`, so jetson with ID 3 should have IP `192.168.1.103` and should be accessible through SSH at `jetson-nano-3@192.168.1.103`. Note that we are actually assigning the IP address to the long-range wifi antennas and not the jetson internal wifi module. We assign the antenna’s MAC address to a static IP within the router’s admin settings. Please ask the project lead if you need the router password. **Each antenna and jetson should have a tag with an ID that it maps to, so don’t interchange antennas and jetsons.**

The master computer should always have the antenna with tag 0 in order to map it to `192.168.1.100`. This is important for roscore and time sync. 

The jetson should automatically connect to `core-robotics-net-2` on boot if the network avialable. You can find all the jetson connected by using `nmap -sP 192.168.1.* | grep "192.168.1.10"`. If a jetson is not connected, you need to ssh into the jetson (likely through ethernet cable) and use `sudo nmcli d wifi connect core-robotics-net-2_5G password ogmcglab`. 

### Time Sync
In order for all the Jetsons to have sync'd time for pose estimates, we use [chrony](https://answers.ros.org/question/298821/tf-timeout-with-multiple-machines/) and let the master computer be the main NTP server.  

### Setup New Jetson
In a 32Gb SD Card, flash the image with all the software requirements from this [link](https://github.com/Qengineering/Jetson-Nano-image). Now to update the username for the jetsons, we can follow the steps given in this [thread](https://forums.developer.nvidia.com/t/changing-the-default-username-on-jetson-tx2/119693/2) and change the home directory with a renamed copy of the original folder. The password should be changed by logging into the root of the system.

To set up the ROS Networking on the Jetsons, change `~/.bashrc` and add: `export ROS_MASTER_URI=http://192.168.1.100:11311` and `export ROS_HOSTNAME=<Jetson IP>`. Make sure Jetson is loaded with all the required packages.

### Calibration:

To get the camera’s relative position within the world, we use [Apriltag](http://wiki.ros.org/apriltag_ros) calibration to get accurate pose estimates. We created a box covered in unique Apriltag bundles so that we can calibrate the cameras with a pre-measured box in a known position in the world. All this functionality is located in the `ball_calibration` package.

Refer to court calibration section for main details. 

### Ball Detection:

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

### Position Tracking:

Currently, for fusing all the camera’s position estimates we use a modified version of `robot_localization`’s EKF that accounts for the ball’s ballistic dynamics called `ball_localization`. We also incorporated the ability to rollout the ball trajectory into the future by iteratively calling the EKF’s predicting method. Setting for the EKF can be tuned within this [yaml](ball_calibration/config/ball_ekf_localization.yaml) based for of `robot_localization`. 

We tune the EKF, especially the ball bounce parameters, using a script that evulates the bag position history versus the EKF rollout for a particular time using this [testing script](ball_calibration/scripts/evalute_rollout.py)

### What to bring to courts:
- 6x Camera Stands (Stand, ZED, Jetson, Battery, Power Cable, Antenna for each)
- Router
- 3x Extension cables
- Calibration Box
- 2 Rackets and a few tennis balls
- Both lab computers 
- Chargers for computers, motor battery, master wheelchair battery
- Master antenna
- WTR Platform
- Tools for wheelchair, duct tape, and colored masking tape
- Ball launcher (optional)
- 2x ethernet cables (optional) (in case jetson breaks)

## Calibration
### Tennis Courts Calibration:
* Set up all jetson stands on the court (ex. Connect jetson batteries, place stands properly, power router). Here is how the stands and box should be setup:
    <p align="center">
        <img src="documents/photos/court_diagram.drawio.png" alt="Court" width="600" />
    </p>
* Set calibration box so its **black** corner aligns with service-line's corner
    <p align="center">
        <img src="documents/photos/box_corner.jpg" alt="Box Corner" width="200" />
    </p>
* Run `nmap -sP 192.168.1.* | grep "192.168.1.10"` to detect available Jetsons (Make sure computer on `core-robotics-net-2`)
* Run Apriltag detection on all cameras:
    * `roslaunch ball_calibration jetson_apriltag_multi.launch`
    * **Note:** Make sure to comment/uncomment broken camera IDs in launch file if some cameras aren't working
    * Check `rqt_gui` that popped up to make sure that tags are getting detected (Press the refresh button if stream not appearing)
* Run calibration script:
  * If you are at courts and set up the cameras properly, you can just do `roslaunch ball_calibration jetson_record_calib.launch type:=our_side` and then with `type:=opp_side` when you move box to opposite side of wheelchair. 
    * Check printed camera statistics to make sure everything is okay
      * All cameras have 5+ samples
  * If at lab, edit `jetson_record_calib.launch` for `type=other` so cameras map to correct bundle 
      * Use colored tape on box for mapping if you want: Bundle 1 (Yellow), Bundle 2 (Green), Bundle 3 (Blue), Bundle 4 (Purple)
    * Then run `roslaunch ball_calibration jetson_record_calib.launch`

    
* Run ball detection on Jetson:
    * `roslaunch ball_calibration jetson_ball_detection_multi.launch`
    * Optional: Tune gain so the images are well-illuminated (Look at _Helpful_ section)
    * You should close the debug image GUI if you want 30 Hz ball pose estimate since producing debug image is a little expensive
* Run `roslaunch ball_calibration box_cameras.launch` or a launch file that includes it like `roslaunch ball_calibration demo_courts.launch`


### How to calibrate standalone zed (without jetsons and box):
* `roslaunch ball_calibration standalone_apriltag.launch` 
* `rosrun tf tf_echo <Parent TF> /zed_camera_center`
    * ex. `<Parent TF>` is frame like `/bundle` or `/tag_5`
* Record pose in `standalone_cameras.launch` for specific camera
* Run ball detection code from `ball_detection` (requires ZED SDK)
    * `roslaunch ball_detection ball_detection.launch`
* Run `roslaunch ball_calibration standalone_cameras.launch`
    * Will run EKF as well

### Helpful
* Run auto-gain calibration script (75 gain is usually good enough):
    * `roslaunch ball_calibration jetson_autogain_multi.launch`
    * Edit `config/jetson_ball_detection.yaml` for respective gain

### Troubleshooting
* Error: **no such option: --sigint-timeout** or **If <filename> is a single dash ('-'),**
    * `sudo gedit /opt/ros/noetic/lib/python3/dist-packages/roslaunch/remoteprocess.py `
    * Edit line 141 to `args = [machine.env_loader, 'roslaunch', '-c', name, '-u', server_uri, '--run_id', run_id]`
* Error: **RLException: Exception while registering with roslaunch parent**
    * Make sure you setup `ROS_MASTER_URI`
* Error: Jetson connecting to other network
  * `sudo nmcli c delete <WIFI Connection>`
  * `sudo nmcli d wifi connect 'core-robotics-net-2_5G' password ogmcglab`


### Comand Cheetsheet
#### At Courts Commands
- `   nmap -sP 192.168.1.* | grep "192.168.1.10"   `
- `   roslaunch ball_calibration jetson_apriltag_multi.launch   `
- `   roslaunch ball_calibration jetson_record_calib.launch type:=court_our   `
- `   roslaunch ball_calibration jetson_record_calib.launch type:=court_opp   `
- `   roslaunch ball_calibration jetson_ball_detection_multi.launch   `
- `   roslaunch wtr_plan demo_court.launch   `

#### Update Jetsons Git Repo Commands
- `  sshpass -p 1111 ssh jetson-nano-1@192.168.1.101  `
- `  cd ~/catkin_ws/src/Wheelchair-Tennis-Robot/ && git pull && cd ~/catkin_ws && catkin_make  `

