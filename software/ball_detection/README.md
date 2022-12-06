# ball_detection Package

Used to detect ball positions in camera frame for use in EKF. See the [ball_calibration](../ball_calibration/README.md) package for more info on setup and usage of this package.

## Note 
Similar to `zed_vision` but with no CUDA dependencies so can build without OpenCV 4.5.2.
Still requires ZED SDK, will print warning message during building if can't find ZED.

This package only runs CPU version of detection code. 
The GPU version is located in `zed_master` branch.

