# Firmware

- [Firmware](#firmware)
  - [Overview](#overview)
  - [Uploading Teensy Sketch](#uploading-teensy-sketch)
  - [Uploading ODrive Config](#uploading-odrive-config)
  - [Remote Control](#remote-control)
    - [Startup](#startup)
    - [Modes](#modes)
    - [Manual Control](#manual-control)
    - [Emergency Stop (E-Stop)](#emergency-stop-e-stop)
    - [Remote Calibration](#remote-calibration)
      - [Positions](#positions)

## Overview

The drivebase of the wheelchair is based on the [ODrive](https://odriverobotics.com/) high performance motor controller with two [ODrive BLDC motors](https://odriverobotics.com/shop/odrive-custom-motor-d6374-150kv) and two [CUI encoders](https://odriverobotics.com/shop/cui-amt-102). The ODrive is controlled by a [Teensy 4.1](https://www.pjrc.com/store/teensy41.html) which handles taking in human inputs from a [remote controller](https://www.amazon.com/dp/B07Q112TQF/) and autonomous inputs from the computer to make control decisions for moving the motors while supplying position and velocity information to the computer for localization. There are several safety features built in like an emergency stop (estop) button on the remote controller for stopping the vehicle or arm if it gets out of control or stops responding. There is a secondary [remote controlled relay](https://www.amazon.com/dp/B07JFH6VQH/) which is used for removing power from the ODrive and motors in case any software stops working.

## Uploading Teensy Sketch

To get the firmware setup, the code needs to be uploaded to the Teensy and the configuration needs to be uploaded to the ODrive. 

By using an Arduino add-on called [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html), we are able to upload the sketch to the Teensy. Start by installing Teensyduino on your computer by following the installation instructions [here](https://www.pjrc.com/teensy/td_download.html). Once installed, upload the [teensy_interface.ino](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/tree/main/firmware/teensy_interface) sketch to the Teensy by plugging it into the computer via USB and setting the Tools > Board info as seen in the top of the teensy_interface.ino file. 

{: .note }
Upon first upload of the sketch to the Teensy, the configuration stored in the EEPROM will not be valid and therefore will expect a remote calibration routine. More infromation about the remote calibration can be found [here]().

## Uploading ODrive Config

To get the ODrive setup to communicate with the Teensy over UART and set up control mode values, we need to upload the configuration file to the ODrive. Start by powering on the ODrive by plugging it into the 48V eBike battery (or plugging it into a wall power supply). Install the [odrivetool](https://docs.odriverobotics.com/v/0.5.5/odrivetool.html) by following the installation instructions [here](https://docs.odriverobotics.com/v/0.5.5/getting-started.html#downloading-and-installing-odriveool). Then plug the ODrive into the computer via USB and upload the [config.json](https://github.com/CORE-Robotics-Lab/Wheelchair-Tennis-Robot/tree/main/firmware/odrive) using the `odrivetool restore-config config.json` command from a terminal. More info on configuration restore can be found [here](https://docs.odriverobotics.com/v/0.5.5/odrivetool.html#configuration-backup).

## Remote Control

The remote controller is the main way of interfacing with the wheelchair. The remote controller allows the user to control the state of robot (idle, manual, or autonomous), driving the robot in manual mode (using throttle and steering), and other features like calibrating the remote and zeroing the location of the robot. The overall remote is shown below.

![Remote Controller](../docs/assets/img/remote_controller/overall.jpeg) 

### Startup
To startup the wheelchair, turn on the wheelchair battery using the power switch on the side of the battery, then use the wireless relay remote to turn on the relay (you should hear an audible click when pressing the "on" button for the first time). Once on, each wheel will rotate up to 1/10th of a rotation to calibrate the each wheel. At that point you can change modes to manual mode or autonomous mode and use the wheelchair as normal.  

### Modes

The wheelchair has three main modes; idle mode, manual mode, and autonomous mode. Idle mode disables the wheelchair motors and allows the wheels to be rotated without any resistance. This is useful for moving the wheelchair around by hand. Manual mode enables the wheelchair motors and allows for control of the wheelchair velocity with the remote controller. See [Manual Control](#manual-control) for more information on remotely controlling the wheelchair. Autonomous mode allows for ROS to control the wheelchair over a USB cable connected to the teensy that creates a rosserial connection and can subscribe and publish to topics on the ROS network. 

| Idle Mode                                                    | Manual Mode                                                    | Autonomous Mode                                                    |
| ------------------------------------------------------------ | -------------------------------------------------------------- | ------------------------------------------------------------------ |
| ![Idle Mode](../docs/assets/img/remote_controller/mode.jpeg) | ![Manual Mode](../docs/assets/img/remote_controller/mode.jpeg) | ![Autonomous Mode](../docs/assets/img/remote_controller/mode.jpeg) |

Some notes on things that happen in each mode, check the table below. 

| Idle Mode                  | Manual Mode                       | Autonomous Mode            |
| -------------------------- | --------------------------------- | -------------------------- |
| Motors are disabled        | Motors are enabled                | Motors are enabled         |
| Velocities are set to zero | Velocities are set to zero        | Velocities are set to zero |
| Light bar set to yellow    | Light bar set to yellow and green | Light bar set to green     |
| ODrive errors are cleared  |                                   |                            |

NOTE: The use of the e-stop will override some of these settings. Read the section on the [e-stop](#emergency-stop-e-stop) for more information on using the e-stop. 

### Manual Control

To manually drive the robot, the index finger trigger is used for linear (forward and reverse) velocity and the wheel is used for angular (left and right) velocity. Below are images demonstrating what each movement of the trigger and wheel do for changing linear and angular velocities. 

| Full Forward Linear Velocity                                                         | Zero Linear Velocity                                                        | Full Reverse Linear Velocity                                                         |
| ------------------------------------------------------------------------------------ | --------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| ![Full Forward Velocity](../docs/assets/img/remote_controller/throttle_forward.jpeg) | ![Zero Velocity](../docs/assets/img/remote_controller/throttle_middle.jpeg) | ![Full Reverse Velocity](../docs/assets/img/remote_controller/throttle_reverse.jpeg) |

| Full Counterclockwise Angular Velocity                                         | Zero Angular Velocity                                                         | Full  Clockwise Angular Velocity                                                 |
| ------------------------------------------------------------------------------ | ----------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| ![Full Left Velocity](../docs/assets/img/remote_controller/steering_left.jpeg) | ![Zero Velocity](../docs/assets/img/remote_controller/steering_straight.jpeg) | ![Full Right Velocity](../docs/assets/img/remote_controller/steering_right.jpeg) |

### Emergency Stop (E-Stop)

To trigger an emergency stop (e-stop), press the button on side of the handle of the remote (labeled CH3). Enabling the e-stop currently sets the velocity of both wheels to zero (which actively tries to stop the wheelchair from moving), removes power from the robotic arm (allowing gravity to slowly let it fall down), and allows for future things to be added (like enabling mechanical brakes). Below are two images of the e-stop while enabled (on the left) and disabled (on the right).

| E-Stop Enabled                                                        | E-Stop Disabled                                                         |
| --------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| ![E-Stop Enabled](../docs/assets/img/remote_controller/estop_on.jpeg) | ![E-Stop Disabled](../docs/assets/img/remote_controller/estop_off.jpeg) |


### Remote Calibration
Over time the zeroed values for each axis of the remote controller can drift. This can be seen on the robot when it moves without any inputs being provided while in manual mode. To trigger a remote calibration, ensure in idle mode and that throttle and steering inputs are not being touched, then rotate the CH5 dial fully counterclockwise and rotate it back to center. Upon entering remote calibration mode, the indicator on the teensy will turn on for a moment while it takes readings of the undisturbed inputs. Once it is done taking readings, the indicator will turn off again. To complete the rest of the calibration, move the inputs to each of the positions listed below, and at each position ensuring the indicator turns on and then back off again before moving on to the next position. 

#### Positions
1. Full Forward Linear Velocity
2. Full Reverse Linear Velocity
3. Full Counterclockwise Angular Velocity
4. Full Clockwise Angular Velocity

Once all of the positions have been reached, the indicator will turn off to let the user know that remote calibration has been completed. To verify that calibration is complete, the user can switch between idle and manual modes and see that the light bar shows the new mode. If the light bar doesn't match the mode selected by the remote controller, restart the system and try remote calibration again. 

