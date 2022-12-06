

// Class that internal state, ROLLOUT initialized

// run a thread, init 30 Hz
// Constantly active
// def rollout(ball_filter_state, measurement.time_, nh, nh_priv):
//     static BallLocalization::RosEkf ball_ekf(nh, nh_priv);
//     static first_call = true
//     if (first_call)
//         ball_ekf.initialize();
//         first_call = false
//     // create copy with desire state without interfercing the normal ball_ekf

//     ball_ekf.setState(ball_filter_state);

//     // make copy of ball_filter (EKF)
//     dt = 1e-3
//     current time
//     // while(time_elapse < 5)
//         // predict(measurement.time_, dt);
//         // nav_msgs::Odometry ball_filteredPosition;
//         // getFilteredOdometryMessage(ball_filteredPosition)
//     // List of odemtry msg
//     // pose, cloud covariance
//     // Publish nav_msg::Path
//     // Pose.seq = in mm the radius of max
//     position

//     // Marker -> path of circles (cov clouds), gradually expanding tube


//     // Custom which postion, velocity, error (confidence)
    // Marker