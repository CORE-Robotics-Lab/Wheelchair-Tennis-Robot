#include "ros/ros.h"
#include <math.h>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <sstream>
#include <iterator>

#include "catapult_swing.cpp"
#include "linear_swing.cpp"
#include "circular_swing.cpp"

// TODO: takeAppend curr -> start, check wheelchair, go there 

// ready? as soon as message arrives
// approx start -> start -> end

/* Typedefs */
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryActionClient;

/* Globals */
std::unique_ptr<TrajectoryActionClient> traj_action_client_;
std::map<std::string, Swing*> swing_type_map_;
ros::ServiceClient traj_profiler_client_;
SwingReponse swing_goal_;
Swing* selected_swing_;
bool is_swing_selected_ = false;;

/* Params */
double hit_time_offset_;

bool hasGoal_ = false;
bool isReady_ = false;

geometry_msgs::PointStamped hitpoint;
ros::Publisher hitpoint_pub;

/* Slowly moves to position */
void moveToPose(Joints positions) {
    wam_control::TrajectoryProfiler profiler_msg;
    profiler_msg.request.end_positions = positions;
    profiler_msg.request.execute = true;
    // profiler_msg.request.vel_cap = 5; //2.5;  // rad/s 
    // profiler_msg.request.acc_cap= 5; //1.0;   // rad/s^2

    profiler_msg.request.vel_cap = 2.5;  // rad/s 
    profiler_msg.request.acc_cap= 1.0;   // rad/s^2

    // Call trajectory profiler to deal with smoothing 
    if (traj_profiler_client_.call(profiler_msg)) {
        if (!profiler_msg.response.success) {
            ROS_INFO("Create trajectory failed!"); 
        }
    } else {
        ROS_INFO("Failed calling trajectory_profiler service");
    }
}

/* Select swing type so can move to approximate start */
void swingSelectCallback(const std_msgs::String::ConstPtr& select_msg) {
    // Msg Example: forehand catapult [forehand/backhand linear/circular/catapult]
    // Split the words
    std::istringstream iss(select_msg->data);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};

    // Error-check tokens and create selected swing 
    if (tokens.size() == 2 
            && (tokens[0] == "forehand" || tokens[0] == "backhand") 
            && (tokens[1] == "linear" || tokens[1] == "circular" || tokens[1] == "catapult")) {

        ROS_INFO_STREAM("Selected " << tokens[0] << " " << tokens[1] << " swing, moving to approximate start joint position.");
        
        selected_swing_ = swing_type_map_[tokens[1]];
        selected_swing_->setShotType(tokens[0]);
        is_swing_selected_ = true;

        // Move to approx start
        moveToPose(selected_swing_->getApproxStart());
    } else {
        ROS_ERROR("Swing selected not valid (ex. forehand catapult [forehand/backhand linear/circular/catapult])");
    }
}

/* Subscriber triggered callback to set isReady flag for swinging */
void readyCallback(const std_msgs::Bool::ConstPtr& ready_msg) {   
    ROS_INFO("Ready to swing...");
    isReady_ = ready_msg->data;
}


bool createTrajectory(wam_control::Swing::Request &req,
                      wam_control::Swing::Response &res) {
    auto start_time = ros::Time::now();

    

    // Create trajectory
    SwingReponse swing_response;
    if (is_swing_selected_) {
        swing_response = selected_swing_->createTrajectory(req);
        res.error_msg = swing_response.error_msg;
    } else {
        res.error_msg = "Unknown swing type!";
    }
    ROS_INFO_STREAM("Create Trajectory service took: " << (ros::Time::now() - start_time).toSec()
                   << "s");

    // Deal with error 
    if (res.error_msg != "") {
        ROS_ERROR_STREAM("Swing Error: " << res.error_msg);
        res.success = false;
        return false;
    }

    // if ((swing_response.start_time-ros::Time::now()).toSec() < 0) {
    //     res.success = false;
    //     res.error_msg = "Hit point will arrive too late!";
    //     return false;
    // }

    ROS_INFO_STREAM("Will try to swing in: " << (swing_response.start_time-ros::Time::now()).toSec() << "s");

    // Save response
    swing_goal_ = swing_response;
    hasGoal_ = true;

    hitpoint = req.ball_position;

    // Successful Return 
    res.success = true;
    res.placement_pose = swing_response.wc_pose;
    return true;
}        



/* Timer triggered callback to determine when to swing */
void ControlCallback(const ros::TimerEvent&) {   
    /* snapshot current swing response */
    SwingReponse curr_swing_goal = swing_goal_;

    /* Check if it is time to swing */
    if (hasGoal_ && isReady_ && ((curr_swing_goal.start_time - ros::Time::now()).toSec() <= hit_time_offset_)) {

        geometry_msgs::PointStamped publish_hitpoint;
        publish_hitpoint.point.x = hitpoint.point.y;
        publish_hitpoint.point.y = hitpoint.point.x;
        publish_hitpoint.point.z = hitpoint.point.z;
        publish_hitpoint.header.frame_id = "world";
        hitpoint_pub.publish(publish_hitpoint);

        /* Send trajectory */
        control_msgs::FollowJointTrajectoryGoal follow_goal;
        follow_goal.trajectory = curr_swing_goal.trajectory;
        
        // Directly execute
        ROS_INFO_STREAM("Sending joint trajectory of " << follow_goal.trajectory.points.size() << " points");
        traj_action_client_->sendGoalAndWait(follow_goal); 
        isReady_ = false;
        hasGoal_ = false;

        ros::Duration(1).sleep();

        // ros::Duration(0.2).sleep();

        // Move to home
        moveToPose(selected_swing_->getApproxStart());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swing_interface");
    ros::NodeHandle nh;
    ros::NodeHandle nhp{"~"};

    ros::AsyncSpinner spinner(2);
    spinner.start();

    hitpoint_pub = nh.advertise<geometry_msgs::PointStamped>("swingserver_hitpoint", 1);

    /* Call backs */
    ros::Timer control_timer = nh.createTimer(ros::Duration(1 / 1000.0), ControlCallback); 
    ros::Subscriber select_swing_sub = nh.subscribe("/swing_ready", 1, readyCallback);
    ros::Subscriber ready_sub = nh.subscribe("/swing_select", 1, swingSelectCallback);

    /* Service servers / clients  */
    ros::ServiceServer create_traj_service = nh.advertiseService("/swing_server", createTrajectory);

    /* Create swing type map */
    traj_profiler_client_ = nh.serviceClient<wam_control::TrajectoryProfiler>("/trajectory_profiler", true);
    swing_type_map_.emplace("linear",   new LinearSwing(traj_profiler_client_));
    swing_type_map_.emplace("circular", new CircularSwing(traj_profiler_client_));
    swing_type_map_.emplace("catapult", new CatapultSwing(traj_profiler_client_));

    /* Params */
    nhp.param("hit_time_offset", hit_time_offset_, 0.0);

    /* Connect to trajectory controller */
    std::string trajectory_server_topic;
    nhp.param("trajectory_server_topic", trajectory_server_topic, std::string("/joint_group_trajectory_controller/follow_joint_trajectory"));
    // joint_trapezoidal_trajectory_controller (for actual robot)
    // joint_group_trajectory_controller (for simulation)
    traj_action_client_ = std::make_unique<TrajectoryActionClient>(trajectory_server_topic, true);
    ROS_INFO_STREAM("Waiting to connect to: " << trajectory_server_topic);
    traj_action_client_->waitForServer();
    ROS_INFO_STREAM("Connected to: " << trajectory_server_topic);

    ROS_INFO("Running swing interface");

    ros::waitForShutdown();
    return 0;
}