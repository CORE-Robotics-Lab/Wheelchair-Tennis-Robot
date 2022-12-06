#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <wam_control/LinearSwing.h>
#include <math.h>


namespace rvt = rviz_visual_tools;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;

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

bool failed(wam_control::LinearSwing::Response& response) {
    response.success = false;
    return true;
}

bool move_callback(wam_control::LinearSwing::Request& request, wam_control::LinearSwing::Response& response) {
    ROS_INFO_NAMED("LinearSwing", "Move Message Recieved");

    geometry_msgs::Point hit_point = request.ball_position.point;
    std::vector<geometry_msgs::Pose> waypoints = makeSwing(hit_point, request.swing_direction, true);

    move_group->setMaxVelocityScalingFactor(1.0); // Probably should be variables
    move_group->setMaxAccelerationScalingFactor(1.0);

    // Visualize Pose Goal
    visual_tools->deleteAllMarkers();
    visual_tools->publishAxisLabeled(waypoints[0], "pose1");
    visual_tools->trigger();

    // Pose goal
    move_group->setPoseTarget(waypoints[0]);
    // move_group->stop();
    if (!move_group->move()) {
        response.success = false;
        return failed(response);
    } else {
        response.success = true;
        return true;
    }
}

bool swing_callback(wam_control::LinearSwing::Request& request, wam_control::LinearSwing::Response& response)
{
    ROS_INFO_NAMED("LinearSwing", "Swing Message Recieved");
    geometry_msgs::Point hit_point = request.ball_position.point;
    std::vector<geometry_msgs::Pose> waypoints = makeSwing(hit_point, request.swing_direction);

    move_group->setMaxVelocityScalingFactor(1.0); // Probably should be variables
    move_group->setMaxAccelerationScalingFactor(1.0);

    // Pose goal
    move_group->setPoseTarget(waypoints[0]);

    // Visualize Pose Goal
    visual_tools->deleteAllMarkers();
    visual_tools->publishAxisLabeled(waypoints[0], "pose1");
    visual_tools->trigger();

    if (!move_group->move()) return failed(response);

    // Cartesian Path
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.00;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    bool cartesian_plan_success = (fraction == 1.0);
    ROS_INFO_NAMED("LinearSwing", "Cartesian path: %.2f%% acheived", fraction * 100.0);
    ROS_INFO_NAMED("LinearSwing", "Planning Cartesian path: %s", cartesian_plan_success ? "SUCCESS" : "FAILED");

    response.percent_complete = fraction;

    if (!cartesian_plan_success) return failed(response);

    // Getting cartesian points from plan (painful)
    // Based on https://answers.ros.org/question/356758/how-to-find-the-end-effector-positionsco-ordinates-for-the-whole-plan-trajectory-in-moveit/
    robot_trajectory::RobotTrajectory robot_traj(move_group->getRobotModel(), "arm");
    robot_traj.setRobotTrajectoryMsg(*(move_group->getCurrentState()), trajectory);
    double end_time = robot_traj.getWayPointDurationFromStart(robot_traj.getWayPointCount()-1);

    robot_state::RobotStatePtr robot_state_ptr(new robot_state::RobotState(move_group->getRobotModel()));

    double hit_time = 0;
    for (double curr_time = 0; curr_time < end_time; curr_time += .01) {
        robot_traj.getStateAtDurationFromStart(curr_time, robot_state_ptr);

        Eigen::Affine3d end_effector_state = robot_state_ptr->getFrameTransform(move_group->getEndEffectorLink());
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(end_effector_state, pose);

        double threshold = .01;

        geometry_msgs::Point curr_point = pose.position;
        if (abs(hit_point.x -  curr_point.x) < threshold
            && abs(hit_point.x -  curr_point.x) < threshold
            && abs(hit_point.x -  curr_point.x) < threshold) {
                hit_time = curr_time;
                break;
        }

    }

    ROS_INFO_NAMED("LinearSwing", "Will hit %f secs into trajectory", hit_time);

    ros::Time goal_time = request.ball_position.header.stamp;
    if (goal_time.sec != 0 && goal_time.nsec != 0) {
        ros::Duration time_until_swing = (goal_time - ros::Time::now()) - ros::Duration(hit_time);
        ROS_INFO_NAMED("LinearSwing", "Will wait %f secs for swing", time_until_swing.toSec());

        if (time_until_swing > ros::Duration(0)) {
            time_until_swing.sleep();
        } else {
            ROS_WARN_NAMED("LinearSwing", "Not Enough Time Until Swing ");
            return failed(response);
        }
    }

    // Visualize the plan in RViz
    visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools->trigger();

    if (!move_group->execute(trajectory)) return failed(response);

    // Response
    response.success = true;
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

    // move_group->setGoalPositionTolerance(0.01);
    // move_group->setGoalOrientationTolerance(0.05);
    move_group->setPlanningTime(0.05);
    move_group->setEndEffectorLink("wam/racquet_hitpoint_link");
    move_group->setMaxVelocityScalingFactor(1.0); // Probably should be variables
    move_group->setMaxAccelerationScalingFactor(1.0);

    // Getting Basic Information
    ROS_INFO_NAMED("LinearSwing", "Planning frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("LinearSwing", "End effector link: %s", move_group->getEndEffectorLink().c_str());

    // Visualization
    visual_tools = std::make_unique<moveit_visual_tools::MoveItVisualTools>("base_footprint");

    ros::ServiceServer swing_service = nh.advertiseService("linear_stroke_swing", swing_callback);
    ros::ServiceServer move_service = nh.advertiseService("linear_stroke_move", move_callback);

    ros::waitForShutdown();
    return 0;
}