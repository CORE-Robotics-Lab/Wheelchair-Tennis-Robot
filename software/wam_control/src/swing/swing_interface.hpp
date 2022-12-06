#pragma once
/* Interface for different types of swings */
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include "wam_control/Swing.h"
#include "wam_control/TrajectoryProfiler.h"
#include "sensor_msgs/JointState.h"

#include "ros/ros.h"

typedef std::vector<double> Joints;

struct SwingReponse {
   bool success;
   std::string error_msg;
   trajectory_msgs::JointTrajectory trajectory;
   ros::Time start_time;
   geometry_msgs::Pose wc_pose;
};

class Swing {
    public:
        Swing(ros::ServiceClient traj_profiler_client) : traj_profiler_client_(traj_profiler_client){
            // Params 
            ros::NodeHandle nh;
            nh.getParam("/max_velocities", max_velocities_);
            nh.getParam("/max_accelerations", max_accelerations_);
            nh.getParam("/min_positions", min_positions_);
            nh.getParam("/max_positions", max_positions_);
            nh.getParam("/joint_names", joint_names_);

            for (int i = 0; i < max_velocities_.size(); i++) {
                assert(max_velocities_[i] != 0);
                assert(max_accelerations_[i] != 0);
                assert(min_positions_[i] <= max_positions_[i]);
                assert(joint_names_[i] != "");
            }

            // Joint Sub
            joints_sub_ = nh.subscribe("/joint_states", 1, &Swing::JointStatesCallback, this);

            // etc.
        }

        // Return success, trajectory, hit_time from swing request
        virtual SwingReponse createTrajectory(wam_control::Swing::Request &req) = 0; 

        // Where should joints should arm roughly start 
        virtual Joints getApproxStart() = 0;

        // Set shot type [ex. forehand, backhand]
        void setShotType(std::string shot_type) { shot_type_ = shot_type; }

     
    protected:
        // Get current joint state
        void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joints_msg) {   
            // Process joint message by re-ordering to base->palm instead of alphabetical
            double joint_num = joints_msg->name.size();
            std::vector<double> curr_positions(joint_num);
            for (int i = 0; i < joint_num; i++) {
                int index = std::find(joint_names_.begin(), joint_names_.end(), joints_msg->name[i]) - joint_names_.begin();
                if (index != joint_names_.size() && index < joint_num) {
                    curr_positions[index] = joints_msg->position[i];
                }
            }
            curr_positions_ = curr_positions;
        }
        
        // Combine two trajectory and update hit time
        std::tuple<trajectory_msgs::JointTrajectory, double> concatTrajectories(
                const trajectory_msgs::JointTrajectory& prefix, const trajectory_msgs::JointTrajectory& suffix, double suffix_hit_time) {
            trajectory_msgs::JointTrajectory new_trajectory = prefix;
            // Shift all points by last element in prefix
            ros::Duration preffix_time_shift = prefix.points[prefix.points.size()-1].time_from_start;
            // std::for_each(suffix.points.begin(), suffix.points.end(), [&](ros::Time& t){ t += preffix_time_shift; });  
            // Combine prefix with suffix
            new_trajectory.points.insert(new_trajectory.points.end(), suffix.points.begin(), suffix.points.end());
            return make_tuple(new_trajectory, preffix_time_shift.toSec() + suffix_hit_time);
        }


        // Global
        ros::ServiceClient traj_profiler_client_; 
        ros::Subscriber joints_sub_;


        // Param
        Joints min_positions_, max_positions_;
        Joints max_accelerations_, max_velocities_;
        std::vector<std::string> joint_names_;
        Joints curr_positions_;

        // State
        std::string shot_type_;
        Joints approx_start_;

};
