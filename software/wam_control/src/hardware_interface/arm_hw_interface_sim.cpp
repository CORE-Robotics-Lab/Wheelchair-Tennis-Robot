// Based on https://github.com/PickNikRobotics/ros_control_boilerplate/blob/noetic-devel/src/generic_hw_interface.cpp
// https://git.barrett.com/software/barrett-ros-pkg/-/blob/devel/wam_node/src/wam_node.cpp
// https://github.com/jhu-lcsr-attic/barrett_control/blob/libbarrett/barrett_hw/src/wam_server.cpp

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

static const int WAM_CONTROL_RATE = 500; // Set libbarrett's WAM control rate
static const int SAFETY_MODE_FREQ =  10; // Safety state feedback rate

namespace barrett_hw
{
  class BarrettHW : public hardware_interface::RobotHW
  {
    public:
          BarrettHW(ros::NodeHandle& nh) : name_("wam_hw_interface"), nh_(nh) {
            // Get URDF
            std::string urdf_string;
            nh.getParam("/robot_description", urdf_string);
            urdf_model_.initString(urdf_string);

            joint_names_ = {
                "wam/base_yaw_joint",
                "wam/shoulder_pitch_joint",
                "wam/shoulder_yaw_joint",
                "wam/elbow_pitch_joint",
                "wam/wrist_yaw_joint",
                "wam/wrist_pitch_joint",
                "wam/palm_yaw_joint"};

            joint_position_.resize(7);
            joint_velocity_.resize(7);
            joint_effort_.resize(7);
            joint_position_command_.resize(7);
            joint_position_true_.resize(7);
            joint_position_prev_.resize(7);

            for (int i = 0; i < joint_names_.size(); i++) {
                ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[i]);

                // Create joint state interface
                state_interface_.registerHandle(hardware_interface::JointStateHandle(
                    joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));

                position_interface_.registerHandle(hardware_interface::JointHandle(
                    state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

                // Check limits
                auto urdf_joint = urdf_model_.getJoint(joint_names_[i]);
                joint_limits_interface::JointLimits joint_limits;
                joint_limits_interface::getJointLimits(urdf_joint, joint_limits);

                pos_jnt_sat_interface_.registerHandle(joint_limits_interface::PositionJointSaturationHandle(
                    position_interface_.getHandle(joint_names_[i]), joint_limits));
            }

            registerInterface(&state_interface_);
            registerInterface(&position_interface_);

            ROS_INFO_STREAM_NAMED(name_, "WAM HW Interface Ready.");
        }

        bool init(ros::NodeHandle& nh, ros::NodeHandle& hw_nh) override { return true; }

        void read(const ros::Time& time, const ros::Duration& period) override {
            for (int i = 0; i < joint_names_.size(); i++) {
                double r = static_cast<double> (rand()) / static_cast<double> (RAND_MAX);
                joint_position_[i] = joint_position_true_[i] + r * 0.0000001; // Need to add a bit of noise so rviz updates


                // Velocity
                static bool first_read = true;
                if (!first_read) {
                    joint_velocity_[i] = (joint_position_[i] - joint_position_prev_[i]) / period.toSec();
                }
                joint_position_prev_[i] = joint_position_[i];
                first_read = false;
            }
        }

        void write(const ros::Time& time, const ros::Duration& period) override {
            for (int i = 0; i < joint_names_.size(); i++) {
                joint_position_true_[i] = joint_position_command_[i];
            }
        }

    private:
        // State
        ros::NodeHandle nh_;
        std::string name_;
        const size_t DOF = 7;

        // States
        std::vector<std::string> joint_names_;

        std::vector<double> joint_position_true_;
        std::vector<double> joint_position_prev_;

        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;

        // Commands
        std::vector<double> joint_position_command_;

        // Configuration
        urdf::Model urdf_model_;

        // ros-controls interface
        hardware_interface::JointStateInterface state_interface_;
        hardware_interface::PositionJointInterface position_interface_;
        joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
  };
}

//wam_main Function
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

    // Construct the wam structure
    ros::NodeHandle barrett_nh("barrett");
    barrett_hw::BarrettHW barrett_robot(barrett_nh);

    // Construct the controller manager
    ros::NodeHandle nh;
    controller_manager::ControllerManager manager(&barrett_robot, nh);

    // Spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Timer variables
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
    ros::Duration period;

    // LOOP
    ros::Rate rate(WAM_CONTROL_RATE);
    while(ros::ok()) {
        // Get the time
        clock_gettime(CLOCK_REALTIME, &ts);
        now.sec = ts.tv_sec;
        now.nsec = ts.tv_nsec;
        period = now - last;
        last = now;

        // Read the state from the WAM
        barrett_robot.read(now, period);

        // Update the controllers
        manager.update(now, period);

        // Write the command to the WAM
        barrett_robot.write(now, period);

        rate.sleep();
    }

    return 0;
}
