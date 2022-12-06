#include "ros/ros.h"
#include <math.h>

#include "wam_control/TrajectoryProfiler.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/GetStateValidity.h>

/* Typedefs */
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;
typedef std::vector<double> Joints;

/* Globals */
std::unique_ptr<TrajectoryActionClient> traj_action_client_;
ros::ServiceClient check_state_validity_client_;

/* ROS params */
Joints curr_positions_;
Joints max_velocities_, max_accelerations_;
Joints min_positions_, max_positions_;
std::vector<std::string> joint_names_;

// Format inspired by: https://git.barrett.com/software/libbarrett/-/blob/f81ee87021305f85151a5b6d829bc78eb5d0e6fb/src/cdlbt/profile.c
struct Profile {
    double p_start;
    double time_end_ramp;
    double p_end_ramp;
    double time_peak;
    double time_start_deramp;
    double p_start_deramp;
    double time_end;
    double p_end;
    double acc;
    double vel;
};

// Get current joint state
void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& joints_msg) {   
    // Process joint message by re-ordering to base->palm instead of alphabetical
    double joint_num = joints_msg->name.size();
    std::vector<double> curr_positions(joint_num), curr_velocities(joint_num);
    for (int i = 0; i < joint_num; i++) {
        int index = std::find(joint_names_.begin(), joint_names_.end(), joints_msg->name[i]) - joint_names_.begin();
        if (index != joint_names_.size() && index < joint_num) {
            curr_positions[index] = joints_msg->position[i];
        }
    }
    curr_positions_ = curr_positions;
}

/* Check if the given trajectory is valid */
bool isValidTrajectory(const trajectory_msgs::JointTrajectory& trajectory) {
    if (!check_state_validity_client_.exists()) { 
        ROS_ERROR("Validity server is not running");
    }

    auto srv_start_time = ros::Time::now();
    moveit_msgs::GetStateValidity check_state_validity_srv;
    check_state_validity_srv.request.group_name = "arm";
    check_state_validity_srv.request.robot_state.joint_state.name = trajectory.joint_names;

    for (int i = 0; i < trajectory.points.size(); i++) {
        check_state_validity_srv.request.robot_state.joint_state.position = trajectory.points[i].positions;
        check_state_validity_srv.request.robot_state.joint_state.velocity = trajectory.points[i].velocities;

        if (check_state_validity_client_.call(check_state_validity_srv)) {
            if (!check_state_validity_srv.response.valid) {
                ROS_ERROR("Invalid trajectory!");
                return false;
            }
        } else {
            ROS_ERROR("Failed calling check_state_validity service");
            return false;
        }
    }
    ROS_INFO_STREAM("Validity checking took: " << (ros::Time::now() - srv_start_time).toSec());
    return true;
}

int sign(double num) {
    return num >= 0 ? 1 : -1; 
}

std::tuple<trajectory_msgs::JointTrajectory, double, double> create_trajectory(
                        Joints start_positions, Joints mid_positions, Joints goal_positions, double vel_cap, double acc_cap) {
    double joint_num = start_positions.size();

    // Find profiles for all joints
    std::vector<Profile> profiles(joint_num);
    for (int i = 0; i < joint_num; i++) {
        ROS_INFO_STREAM("Joint " << i);

        ROS_INFO_STREAM("Acceleration cap: " << acc_cap << " rad/s^2");
        ROS_INFO_STREAM("Velocity cap: " << vel_cap << " rad/s");


        double start_pos = start_positions[i];
        double mid_pos = mid_positions[i];
        double end_pos = goal_positions[i];
        double delta_pos = end_pos - start_pos;

        

        // Find max velocities and acceleration possible for the given range of motion
        double max_acc = sign(delta_pos) * std::min(max_accelerations_[i], acc_cap);
        double max_vel_init2mid = sqrt(2*max_acc*(mid_pos - start_pos));
        double max_vel_mid2end = sqrt(2*max_acc*(end_pos - mid_pos));
        double max_vel = sign(delta_pos) * std::min({max_velocities_[i], max_vel_init2mid, max_vel_mid2end, vel_cap});

        ROS_INFO_STREAM("max_acc:  " << max_acc << " rad/s^2");
        ROS_INFO_STREAM("max_vel_init2mid:  " << max_vel_init2mid << " rad/s");
        ROS_INFO_STREAM("max_vel_mid2end:  " << max_vel_mid2end << " rad/s");
        ROS_INFO_STREAM("max_vel:  " << max_vel << " rad/s");

        /* Find profile for joint trajectory: ex. ---, /\, /```\ */
        Profile& profile = profiles[i];
        profile.p_start = start_pos;
        profile.time_end_ramp = max_vel / max_acc;
        profile.p_end_ramp = start_pos + (max_acc / 2.0) * std::pow(profile.time_end_ramp, 2);
        profile.p_start_deramp = end_pos - std::pow(max_vel, 2) / (2 * max_acc);  //  x_start = x_end - v^2 / 2a
        profile.time_start_deramp = profile.time_end_ramp + (max_vel != 0  ? abs((profile.p_start_deramp - profile.p_end_ramp) / max_vel) : 0);
        profile.p_end = end_pos;
        profile.time_end = profile.time_start_deramp + max_vel / max_acc;
        profile.time_peak = profile.time_end_ramp + (profile.time_start_deramp - profile.time_end_ramp) / 2.0;
        profile.acc = max_acc;
        profile.vel = max_vel;

        ROS_INFO_STREAM("p_start:  " << profile.p_start << " rad");
        ROS_INFO_STREAM("time_end_ramp:  " << profile.time_end_ramp << " sec");
        ROS_INFO_STREAM("p_end_ramp:  " << profile.p_end_ramp << " rad");
        ROS_INFO_STREAM("p_start_deramp:  " << profile.p_start_deramp << " rad");
        ROS_INFO_STREAM("time_start_deramp:  " << profile.time_start_deramp << " sec");
        ROS_INFO_STREAM("p_end:  " << profile.p_end << " rad");
        ROS_INFO_STREAM("time_end:  " << profile.time_end << " sec");
        ROS_INFO_STREAM("time_peak:  " << profile.time_peak << " sec");
        ROS_INFO_STREAM("acc:  " << profile.acc << " rad/s^2");
        ROS_INFO_STREAM("max_vel:  " << profile.vel << " rad/s");

        ROS_INFO_STREAM("============================================");
    }

    // Find longest profile
    Profile longest_profile = *std::max_element(profiles.begin(), profiles.end(), 
                                [](const Profile& a, const Profile& b) { return a.time_end < b.time_end; });

    ROS_INFO_STREAM("!!!!!!!!!!!!");
    ROS_INFO_STREAM("Longest Profile time peak:  " << longest_profile.time_peak << "s");
    ROS_INFO_STREAM("Longest Profile time end:  " << longest_profile.time_end << "s");
    ROS_INFO_STREAM("Longest profile's acc:  " << longest_profile.acc << " rad/s^2");
    ROS_INFO_STREAM("Longest profile's max_vel:  " << longest_profile.vel << " rad/s");
    ROS_INFO_STREAM("!!!!!!!!!!!!");

    // Shift times and find unique event times
    auto cmp = [](double t1, double t2){ return  round(t1*1.e6)/1.e6 < round(t2*1.e6)/1.e6; }; // reduce precision because issues with repeated times 
    std::set<double, decltype(cmp)> unique_times;
    int i = 0;
    for (const Profile& p : profiles) {
        double time_shift = longest_profile.time_peak - p.time_peak;
        unique_times.insert({time_shift, time_shift + p.time_end_ramp, time_shift + p.time_start_deramp, time_shift + p.time_end});

        ROS_INFO_STREAM("time_shift:  " << time_shift << " s");        
    }

    // Create trajectory
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;

    for(double time : unique_times) {
        ROS_INFO_STREAM("unique time:  " << time << " s"); 
        Joints traj_positions(joint_num), traj_velocities(joint_num), traj_accelerations(joint_num);
        for (int i = 0; i < joint_num; i++) {
            Profile p = profiles[i];
            double& pos = traj_positions[i];
            double& vel = traj_velocities[i];
            double& acc = traj_accelerations[i];
            double adjusted_t = time - (longest_profile.time_peak - p.time_peak);

            ROS_INFO_STREAM("adjusted t:  " << adjusted_t << " s"); 

            if (adjusted_t < 0.0) { 
                pos = p.p_start;
                vel = 0;
                acc = 0;
            } else if (adjusted_t < p.time_end_ramp) {     // Ramp up
                pos = p.p_start + (p.acc/2.0) * std::pow(adjusted_t, 2);
                vel = p.acc * adjusted_t;
                acc = p.acc;
            } else if (adjusted_t >= p.time_end_ramp && adjusted_t <= p.time_start_deramp) { // Plateau
                pos = p.p_end_ramp + p.vel * (adjusted_t - p.time_end_ramp);
                vel = p.vel;
                acc = 0;
            } else if (adjusted_t < p.time_end) {          // Ramp down
                pos = p.p_end - (p.acc/2.0) * std::pow(adjusted_t - p.time_end, 2);
                vel = -p.acc * (adjusted_t- p.time_end);
                acc = -p.acc;
            } else {
                pos = p.p_end;
                vel = 0;
                acc = 0;
            }
        } 

        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(time);
        point.positions = traj_positions;
        point.velocities = traj_velocities;
        point.accelerations = traj_accelerations;
        trajectory.points.push_back(point);
    }

    return std::make_tuple(trajectory, longest_profile.time_peak, longest_profile.time_end);
}

bool trajectory_profiler(wam_control::TrajectoryProfiler::Request &req,
                         wam_control::TrajectoryProfiler::Response &res)
{
    auto start_time = ros::Time::now();

    /* Get beg, mid, and end joints (change to defaults if unpopulated) */
    Joints beg_jnts = (!req.beg_positions.empty()) ? req.beg_positions : curr_positions_;
    Joints mid_jnts = (!req.mid_positions.empty()) ? req.mid_positions : Joints(7);
    Joints end_jnts = req.end_positions;
    if (req.mid_positions.empty()) {
        for (int i = 0; i < std::min(beg_jnts.size(), end_jnts.size()); i++) {
            mid_jnts[i] = (end_jnts[i] + beg_jnts[i]) / 2.0;
        }
    }

    /* Safety checks */
    if (beg_jnts.size() != mid_jnts.size() && mid_jnts.size() != end_jnts.size()) { // Check Same Length
        ROS_ERROR_STREAM("start, mid, end joints sizes not same: " << beg_jnts.size() << " != " << mid_jnts.size() << " != " << end_jnts.size());
        return false;
    } else if (beg_jnts.empty()) {                                                  // Check if empty
        ROS_ERROR_STREAM("Command joint positions are empty");
        return false;
    } 
    int joint_num = end_jnts.size();
    for (int i = 0; i < joint_num; i++) {
        if ((min_positions_[i] > beg_jnts[i] || beg_jnts[i] > max_positions_[i])    // Check joint limits
        && (min_positions_[i] > end_jnts[i] || end_jnts[i] > max_positions_[i]))  {
            ROS_ERROR("Command start/end joint outside joint limits");
        }
        if ( !((beg_jnts[i] <= mid_jnts[i] &&  mid_jnts[i] <= end_jnts[i])          // Check mid in middle 
            || (end_jnts[i] <= mid_jnts[i] &&  mid_jnts[i] <= beg_jnts[i])) ) {
            ROS_ERROR("Command middle joint not in middle");
        }
    }

    double vel_cap = req.vel_cap > 0 ? req.vel_cap : INT_MAX;
    double acc_cap = req.acc_cap > 0 ? req.acc_cap : INT_MAX;
    
    /* Create Trajectory */
    auto[trajectory, mid_arrival, end_arrival] = create_trajectory(beg_jnts, mid_jnts, end_jnts, vel_cap, acc_cap);
    trajectory.joint_names = joint_names_;

    /* Check trajectory validity */
    bool valid = true;
    if (req.check_validity) {
        valid = isValidTrajectory(trajectory);
    }

    ROS_INFO_STREAM("Trajectory Duration " << (ros::Time::now() - start_time).toSec() << " sec");
    
    /* Send trajectory */
    if (valid && req.execute) {
        ROS_INFO_STREAM("Sending Joint");
        control_msgs::FollowJointTrajectoryGoal trajectory_goal;
        trajectory_goal.trajectory = trajectory;
        traj_action_client_->sendGoalAndWait(trajectory_goal);
    }

    res.trajectory = trajectory;
    res.mid_arrival = mid_arrival;
    res.end_arrival = end_arrival;
    res.success = valid;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_profiler_server");
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    nh.getParam("/max_velocities", max_velocities_);
    nh.getParam("/max_accelerations", max_accelerations_);
    nh.getParam("/min_positions", min_positions_);
    nh.getParam("/max_positions", max_positions_);
    nh.getParam("/joint_names", joint_names_);

    std::string trajectory_server;
    nhp.getParam("trajectory_server", trajectory_server);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Connect to trajectory controller
    traj_action_client_ = std::make_unique<TrajectoryActionClient>(trajectory_server, true);
    ROS_INFO_STREAM("Waiting to connect to: " << trajectory_server);
    traj_action_client_->waitForServer();
    ROS_INFO_STREAM("Connected to: " << trajectory_server);

    ros::Subscriber joints_sub = nh.subscribe("/joint_states", 1, JointStatesCallback);
    ros::ServiceServer service = nh.advertiseService("/trajectory_profiler", trajectory_profiler);
    check_state_validity_client_ = nh.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity", true);

    ROS_INFO("Ready to generate trajectories with trapezoidal velocity profile");

    ros::waitForShutdown();
    return 0;
}