#include "swing_interface.hpp"
#include "ros/ros.h"

class LinearSwing : public Swing { 
    public:
        LinearSwing(ros::ServiceClient traj_profiler_client) : Swing(traj_profiler_client) { }

        SwingReponse createTrajectory(wam_control::Swing::Request &req) override {
            geometry_msgs::PointStamped ball_position = req.ball_position;        
            // TODO: Implement Me!
            SwingReponse reponse;
            return reponse;
        }

        Joints getApproxStart() { return approx_start_; }

};
