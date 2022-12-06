#include "swing_interface.hpp"
#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <tf2_ros/transform_listener.h>

struct TrajectoryOutline {
    Joints beg_jnts;
    Joints mid_jnts;
    Joints end_jnts;
    geometry_msgs::Pose wc_pose;
    bool success;
    std::string error_msg;
};

class CatapultSwing : public Swing { 
    public:
        CatapultSwing(ros::ServiceClient traj_profiler_client) : Swing(traj_profiler_client) { 
            // etc.
        }

        SwingReponse createTrajectory(wam_control::Swing::Request &req) {
            SwingReponse response;

            // Outline Trajectory
            TrajectoryOutline outline = outlineTrajectory(req.ball_position.point);
            if (!outline.success) {
                response.success = outline.success;
                response.error_msg = outline.error_msg;
                return response;
            }
            response.wc_pose = outline.wc_pose;

            // Create [start -> mid -> end] trajectory 
            wam_control::TrajectoryProfiler swing_profiler_msg;
            swing_profiler_msg.request.beg_positions = outline.beg_jnts;
            swing_profiler_msg.request.mid_positions = outline.mid_jnts;
            swing_profiler_msg.request.end_positions = outline.end_jnts;

            trajectory_msgs::JointTrajectory swing_trajectory;
            double swing_mid_arrival;
            if (traj_profiler_client_.call(swing_profiler_msg)) {
                if (!swing_profiler_msg.response.success) {
                    response.success = false;
                    response.error_msg = "Create trajectory failed!"; 
                    return response;
                }
                swing_trajectory = swing_profiler_msg.response.trajectory;
                swing_mid_arrival = swing_profiler_msg.response.mid_arrival;
            } else {
                response.success = false;
                response.error_msg = "Failed calling trajectory_profiler service"; 
                return response;
            }

            // Create [approx start -> start] trajectroy
            wam_control::TrajectoryProfiler prefix_profiler_msg;
            prefix_profiler_msg.request.beg_positions = approx_start_;
            prefix_profiler_msg.request.mid_positions = Joints();
            prefix_profiler_msg.request.end_positions = outline.beg_jnts;

            trajectory_msgs::JointTrajectory prefix_trajectory;
            if (traj_profiler_client_.call(prefix_profiler_msg)) {
                if (!prefix_profiler_msg.response.success) {
                    response.success = false;
                    response.error_msg = "Create trajectory failed!"; 
                    return response;
                }
                prefix_trajectory = prefix_profiler_msg.response.trajectory;
            } else {
                response.success = false;
                response.error_msg = "Failed calling trajectory_profiler service"; 
                return response;
            }

            // auto[trajectory, updated_hit_time] = concatTrajectories(prefix_trajectory, swing_trajectory, swing_mid_arrival);

            response.trajectory = swing_trajectory;
            response.start_time =  req.ball_position.header.stamp - ros::Duration(swing_mid_arrival);

            response.success = true;
            return response;
        }

        TrajectoryOutline outlineTrajectory(geometry_msgs::Point hitpoint) {
            TrajectoryOutline outline;

            // Got the following two values from tf instead of manually assigning
            const double SHOULDER_Z_OFFSET = 0.837;    // wam shoulder pitch joint origin offset in the z direction from the robot base footprint
            const double SHOULDER_X_OFFSET = 0.079;    // wam shoulder pitch joint origin offset in the x direction from the robot base footprint
            const double ARM_LENGTH = 1.42; //1.420;           // arm length from shoulder pitch joint origin to center of racket calculated manually
            
            const double MAX_PITCH_ANGLE = M_PI_2;      // maximum angle which the arm can make while doing it's swing (at end of swing it can be be completely upright)
            const double MIN_PITCH_ANGLE = 0.0;         // minimum angle which the arm can make while doing it's swing (at end of swing it can be be completely upright)
            const double SHOULDER_PITCH_OFFSET = 0.3;   // angle offset that the arm travels in the either direction before and after hitting the ball i.e. midposition
            const double BASE_YAW_OFFSET = 0.0; //0.3;         // since we have tolerance on the base yaw joint, we can move it more to increase our chances of hittong the ball at maximum velocity
            const double WRIST_YAW_OFFSET = 0.9;         

            double z = hitpoint.z;
            double y = hitpoint.y;

            // Check that ball is high enough
            // if (!( (SHOULDER_Z_OFFSET + ARM_LENGTH*sin(MIN_PITCH_ANGLE + SHOULDER_PITCH_OFFSET) <= z) 
            //     && ( z <= SHOULDER_Z_OFFSET + ARM_LENGTH*sin(MAX_PITCH_ANGLE - SHOULDER_PITCH_OFFSET)) ))
            if (!( 0.839 <= z) && ( z <= SHOULDER_Z_OFFSET + ARM_LENGTH - 0.1)){
                outline.success = false;
                outline.error_msg = "ball not within the lower and upper height limits";
                return outline;
            }

            int joint_num = min_positions_.size();
            Joints beg_jnts(joint_num),  mid_jnts(joint_num), end_jnts(joint_num);
            double dir = (shot_type_ == "forehand") ? 1 : -1;

            /* base_yaw_joint: Based on joint limits */
            double yaw_acc = dir * max_accelerations_[0];
            double yaw_vel = std::min({max_velocities_[0], 
                                       sqrt(2*abs(yaw_acc * min_positions_[0])), 
                                       sqrt(2*abs(yaw_acc * max_positions_[0])) });
            beg_jnts[0] = mid_jnts[0] - (pow(yaw_vel, 2)/(2*yaw_acc)) - (dir * BASE_YAW_OFFSET);
            mid_jnts[0] = 0;
            end_jnts[0] = mid_jnts[0] + (pow(yaw_vel, 2)/(2*yaw_acc)) - (dir * BASE_YAW_OFFSET);

            ROS_INFO_STREAM("Base Yaw Approx Start: " << beg_jnts[0]);

            /*  elbow_pitch_joint: Based on joint limits */
            double pitch_acc = dir * max_accelerations_[3];
            double pitch_vel = std::min({max_velocities_[3], 
                                         sqrt(2*abs(pitch_acc * min_positions_[3])), 
                                         sqrt(2*abs(pitch_acc * max_positions_[3])) });
            beg_jnts[3] = mid_jnts[3] + (pow(pitch_vel, 2)/(2*pitch_acc));
            mid_jnts[3] = 0;
            end_jnts[3] = mid_jnts[3] - (pow(pitch_vel, 2)/(2*pitch_acc));

            /* shoulder_pitch_joint: Based on ball pitch angle */
            double ball_pitch_angle = asin((z - SHOULDER_Z_OFFSET)/ARM_LENGTH);  // Above horizontal 
            mid_jnts[1] = -MAX_PITCH_ANGLE + ball_pitch_angle + 0.1;// + 0.25;
            beg_jnts[1] = -MAX_PITCH_ANGLE; //mid_jnts[1] - SHOULDER_PITCH_OFFSET;
            end_jnts[1] = std::min({mid_jnts[1] + ball_pitch_angle + 0.25, MAX_PITCH_ANGLE - 0.523}); //SHOULDER_PITCH_OFFSET + 0.25;
            
            /* shoulder_yaw_joint & wrist_yaw_joint  */
            beg_jnts[2] = mid_jnts[2] = end_jnts[2] = M_PI_2; 
            beg_jnts[4] = mid_jnts[4] = end_jnts[4] = -dir*WRIST_YAW_OFFSET; //dir*WRIST_YAW_OFFSET; catapult
            // beg_jnts[4] = mid_jnts[4] = end_jnts[4] = dir*WRIST_YAW_OFFSET; //linearish catapult
            
            /* wrist_pitch_joint */
            beg_jnts[5] = mid_jnts[5] = end_jnts[5] = 0;
            // beg_jnts[5] = -dir*M_PI_2/2;  //linearish catapult
            // mid_jnts[5] = 0;          //linearish catapult
            // end_jnts[5] = dir*M_PI_2/2;   //linearish catapult

            /* palm_yaw_joint  */
            beg_jnts[6] = mid_jnts[6] = end_jnts[6] = 0;

            // Find Wheelchair position
            tf2::Quaternion wcQuaternion;
            wcQuaternion.setRPY(0, 0, -dir * M_PI_2);
            wcQuaternion = wcQuaternion.normalize();
            geometry_msgs::Pose wc_pose;
            wc_pose.orientation.x = wcQuaternion.x();
            wc_pose.orientation.y = wcQuaternion.y();
            wc_pose.orientation.z = wcQuaternion.z();
            wc_pose.orientation.w = wcQuaternion.w();
            wc_pose.position.y = y + dir * (SHOULDER_X_OFFSET + ARM_LENGTH*cos(ball_pitch_angle));

            // Create Result
            outline.success = true;
            outline.error_msg = "";
            outline.beg_jnts = beg_jnts;
            outline.mid_jnts = mid_jnts;
            outline.end_jnts = end_jnts;
            outline.wc_pose = wc_pose;
            return outline;
        }

        // Where should joints should arm roughly start 
        Joints getApproxStart() { 
            // Create Approx Start
            geometry_msgs::Point hitpoint;
            hitpoint.z = 1.3;
            TrajectoryOutline outline = outlineTrajectory(hitpoint);
            if (outline.success) {
                approx_start_ = outline.beg_jnts;
            } else {
                throw std::invalid_argument("Catapult's approximate start joint positions errored out");
            }

            ROS_INFO_STREAM("Base Yaw Approx Start: " << outline.beg_jnts[0]);
            
            return approx_start_; 
        };

};


