#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <wam_control/Swing.h>
#include <math.h>


namespace rvt = rviz_visual_tools;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
std::unique_ptr<moveit::core::JointModelGroup> joint_model_group;


moveit_msgs::RobotTrajectory trajectory_;
ros::Time swing_time_;
bool have_goal_;


std::vector<geometry_msgs::Pose> makeSwing(const geometry_msgs::Point& point,
        const geometry_msgs::Vector3& dir, bool justBase=false) {
    std::vector<geometry_msgs::Pose> waypoints;
    double distance = 0.3;

    double norm = abs(dir.x) + abs(dir.y) + abs(dir.z);

    double pitch_slope = sqrt(pow(dir.x, 2) + pow(dir.y, 2)) / dir.z;
    double yaw_slope = dir.y / dir.x;

    double pitch_angle = atan(pitch_slope);
    double yaw_angle = atan(yaw_slope);
    // TODO: Add different rolls for each type of swing (based on orientation)
    double roll_angle = point.y >= 0 ? 0 : M_PI;

    geometry_msgs::Pose base_point;
    base_point.position = point;
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(roll_angle, pitch_angle, yaw_angle), base_point.orientation);

    if (justBase) {
        waypoints.push_back(base_point);

        return waypoints;
    }

    geometry_msgs::Pose pose1 = base_point;
    pose1.position.x -= distance * (dir.x / norm);
    pose1.position.y -= distance * (dir.y / norm);
    pose1.position.z -= distance * (dir.z / norm);
    waypoints.push_back(pose1);

    waypoints.push_back(base_point);

    geometry_msgs::Pose pose3 = base_point;
    pose3.position.x += distance * (dir.x / norm);
    pose3.position.y += distance * (dir.y / norm);
    pose3.position.z += distance * (dir.z / norm);
    waypoints.push_back(pose3);

    return waypoints;
}

bool failed(wam_control::Swing::Response& response) {
    response.success = false;
    return true;
}

bool close_enough(const geometry_msgs::Point& a, const geometry_msgs::Point& b, double tolerance) {
    return abs(a.x - b.x) < tolerance && abs(a.y - b.y) < tolerance && abs(a.z -  b.z) < tolerance;
}

bool swing_callback(wam_control::Swing::Request& request, wam_control::Swing::Response& response)
{
    ROS_INFO("[Swing] Swing Message Recieved");
    geometry_msgs::Point hit_point = request.ball_position.point;
    std::string swing_type = request.swing_type;
    geometry_msgs::Vector3 direction = request.swing_direction;

    // Get Cartesian Path
    std::vector<geometry_msgs::Pose> waypoints = makeSwing(hit_point, direction);

    // Visualize the plan in RViz
    visual_tools->deleteAllMarkers();
    visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::MEDIUM);
    visual_tools->trigger();

    // Generate Joint Trajectory
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.00, trajectory);
    bool cartesian_plan_success = (fraction == 1.0);
    ROS_INFO("[Swing] Cartesian path: %.2f%% acheived", fraction * 100.0);
    // response.percent_complete = fraction;

    if (cartesian_plan_success) {
        ROS_INFO("[Swing] Planning Cartesian path: SUCCESS");
    } else {
        ROS_ERROR("[Swing] Planning Cartesian path: FAILED");
        return failed(response);
    }

    // Get Init Joint
    // trajectory, point, joint

    // Find when make contact
    // Getting cartesian points from plan is painful
    // Based on https://answers.ros.org/question/356758/how-to-find-the-end-effector-positionsco-ordinates-for-the-whole-plan-trajectory-in-moveit/
    robot_trajectory::RobotTrajectory robot_traj(move_group->getRobotModel(), "arm");
    robot_state::RobotStatePtr robot_state_ptr(new robot_state::RobotState(move_group->getRobotModel()));


    robot_traj.setRobotTrajectoryMsg(*(move_group->getCurrentState()), trajectory);
    double end_time = robot_traj.getWayPointDurationFromStart(robot_traj.getWayPointCount()-1);


    double hit_time = 0;
    for (double curr_time = 0; curr_time < end_time; curr_time += .01) {
        robot_traj.getStateAtDurationFromStart(curr_time, robot_state_ptr);

        Eigen::Affine3d end_effector_state = robot_state_ptr->getFrameTransform(move_group->getEndEffectorLink());
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);

        geometry_msgs::Point curr_point = pose.position;

        // geometry_msgs::Point init_point = waypoints[0].position;

        // if (close_enough(curr_point, init_point, 0.01)) {
        //     std::vector<double> init_joint_positions;
        //     robot_state_ptr->copyJointGroupPositions(joint_model_group, init_joint_positions);
        // }

        if (close_enough(curr_point, hit_point, 0.01)) {
            hit_time = curr_time;
            break;
        }
    }

    ROS_INFO("[Swing] Will hit %f secs into trajectory", hit_time);

    // Get ros Time of contact
    ros::Time goal_time = request.ball_position.header.stamp;
    ros::Time swing_time;
    if (goal_time.sec != 0 && goal_time.nsec != 0) {
        swing_time = goal_time - ros::Duration(hit_time);
        ros::Duration time_until_swing = swing_time - ros::Time::now();

        if (time_until_swing.toSec() <= 0) {
            ROS_ERROR("[Swing] Not Enough Time Until Swing!");
            return failed(response);
        }
        ROS_INFO("[Swing] Will wait %f secs for swing", time_until_swing.toSec());

    } else {
        swing_time = ros::Time::now();
    }

    // Update Goal
    trajectory_ = trajectory;
    swing_time_ = swing_time;
    have_goal_ = true;

    // Response
    response.success = cartesian_plan_success;
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear_swing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    // joint_model_group = std::make_unique<const JointModelGroup>(move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP));

    move_group->setGoalPositionTolerance(0.01);
    move_group->setGoalOrientationTolerance(0.05);
    move_group->setPlanningTime(0.1);
    move_group->setEndEffectorLink("wam/racquet_hitpoint_link");
    move_group->setMaxVelocityScalingFactor(1); // Probably should be variables
    move_group->setMaxAccelerationScalingFactor(1);

    // Getting Basic Information
    ROS_INFO("[Swing] Planning frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO("[Swing] End effector link: %s", move_group->getEndEffectorLink().c_str());

    // Visualization
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base_footprint");

    ros::ServiceServer swing_service = nh.advertiseService("stroke_swing_service", swing_callback);

    ros::Rate rate(100);
    while (ros::ok()) {
        // Update Plan
        rate.sleep();
        ros::spinOnce();

        if (!have_goal_)
            continue;

        // If time to swing, swing
        if (ros::Time::now() >= swing_time_) {
            ROS_INFO("[Swing] Swinging!");
            have_goal_ = false;
            if (!move_group->execute(trajectory_)) {
                ROS_ERROR("[Swing] Failed in executing Swing!");
            }
        }
    }

    return 0;
}