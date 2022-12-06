#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "TennisSwingLUT.h"

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


std::vector<geometry_msgs::Pose> LUTtoVector() {
    int motion_id = 0;
    std::vector<geometry_msgs::Pose> waypoints;
    for(int pnt_idx = 0; pnt_idx < TennisLUTLength; pnt_idx+=1) {
        geometry_msgs::Pose pose;
        pose.position.x = TennisLUT[motion_id][pnt_idx][1] ;
        pose.position.y = -TennisLUT[motion_id][pnt_idx][2] ;
        pose.position.z = TennisLUT[motion_id][pnt_idx][3] ;
        // pose.orientation.w = 1;
        pose.orientation.w = TennisLUT[motion_id][pnt_idx][4];
        pose.orientation.x = TennisLUT[motion_id][pnt_idx][5];
        pose.orientation.y = TennisLUT[motion_id][pnt_idx][6];
        pose.orientation.z = TennisLUT[motion_id][pnt_idx][7];

        tf::Pose pose_tf;
        tf::poseMsgToTF(pose, pose_tf);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0.346));
        transform.setRotation(tf::createQuaternionFromRPY(0, M_PI, 0));

        tf::Pose transformed_pose_tf = transform * pose_tf;

        geometry_msgs::Pose transformed_pose;
        tf::poseTFToMsg(transformed_pose_tf, transformed_pose);

        waypoints.push_back(transformed_pose);
    }

    return waypoints;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setGoalPositionTolerance(.01);
    move_group.setGoalOrientationTolerance(1);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Getting Basic Information
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    move_group.setMaxVelocityScalingFactor(.2);

    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // .. _move_group_interface-planning-to-pose-goal:
    //
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    std::vector<geometry_msgs::Pose> waypoints = LUTtoVector();

    move_group.setPoseTarget(waypoints[0]);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(waypoints[0], "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    // move_group.move();


    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.00;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (tolerance) (%.2f acheived)", move_group.getGoalPositionTolerance());

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // move_group.execute(trajectory);
    // move_group.move();


    ros::shutdown();
    return 0;
}