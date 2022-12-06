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
#include <algorithm>

#include <wtr_navigation/WheelInfo.h>
#include <std_msgs/Float32.h>

#include <control_toolbox/pid.h>

namespace drive_hw
{
  class DriveHW : public hardware_interface::RobotHW
  {
    public:
        void currVelocitiesCB(const wtr_navigation::WheelInfo::ConstPtr& velocity_msg) {   
            velocity_[0] = velocity_msg->left;
            velocity_[1] = velocity_msg->right;
        }
        void currPositionsCB(const wtr_navigation::WheelInfo::ConstPtr& position_msg) {
            position_[0] = position_msg->left;
            position_[1] = position_msg->right;
        }

        DriveHW(ros::NodeHandle& nh, bool sim_flag, bool wc_js_flag) : nh_(nh), sim_flag_(sim_flag), wc_js_flag_(wc_js_flag) {
            position_ = std::vector<double>(2, 0);
            velocity_= std::vector<double>(2, 0);
            effort_= std::vector<double>(2, 0);
            velocity_command_= std::vector<double>(2, 0);

            command_vel_pub_ = nh_.advertise<wtr_navigation::WheelInfo>("/wheel_cmd_vel",10);
            pos_sub_ = nh_.subscribe("/wheel_est_pos", 10, &DriveHW::currPositionsCB, this);
            vel_sub_ = nh_.subscribe("/wheel_est_vel", 10, &DriveHW::currVelocitiesCB, this);

            state_interface_.registerHandle(hardware_interface::JointStateHandle("wheel_left_joint", &position_[0], &velocity_[0], &effort_[0]));
            velocity_interface_.registerHandle(hardware_interface::JointHandle(state_interface_.getHandle("wheel_left_joint"), &velocity_command_[0]));

            state_interface_.registerHandle(hardware_interface::JointStateHandle("wheel_right_joint", &position_[1], &velocity_[1], &effort_[1]));
            velocity_interface_.registerHandle(hardware_interface::JointHandle(state_interface_.getHandle("wheel_right_joint"), &velocity_command_[1]));

            registerInterface(&state_interface_);
            registerInterface(&velocity_interface_);

            ROS_INFO_STREAM_NAMED("drive_hw_interface", "Drive HW Interface Ready.");
        }

        void read(const ros::Time& time, const ros::Duration& period) override {
            if (sim_flag_){
                //if simulation flag is set then set the wheel estimated velocities to the commanded velocity
                velocity_[0] = velocity_command_[0];
                velocity_[1] = velocity_command_[1];
                //finding the corresponding position values
                position_[0] += velocity_[0] * period.toSec();
                position_[1] += velocity_[1] * period.toSec();
            }
        }

        void write(const ros::Time& time, const ros::Duration& period) override {
            wtr_navigation::WheelInfo command_vel;
            command_vel.left = velocity_command_[0];
            command_vel.right = velocity_command_[1];

            if (!wc_js_flag_){
                //don't publish if using wheelchair joystick in autonomous mode
                // ROS_INFO_STREAM("PUBLISHING COMMANDED VELOCITY AS WHEELCHAIR JOYSTICK MODE OFF");
                command_vel_pub_.publish(command_vel);
            }
            
         }

    private:
        // State
        ros::NodeHandle nh_;

        //simulation flag
        bool sim_flag_;

        //wheelchair joystick flag
        bool wc_js_flag_;
        
        // States
        std::vector<double> position_;
        std::vector<double> velocity_;
        std::vector<double> effort_;

        // Commands
        std::vector<double> velocity_command_;
        
        // Pub / Sub
        ros::Publisher command_vel_pub_;
        ros::Subscriber pos_sub_;
        ros::Subscriber vel_sub_;

        // ros-controls interface
        hardware_interface::JointStateInterface state_interface_;
        hardware_interface::VelocityJointInterface velocity_interface_;
  };
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_hw_interface");

    // Getting Simulation flag
    ros::NodeHandle nhp{"~"};
    bool sim_flag;
    nhp.getParam("sim_drive_controller", sim_flag);

    // Getting Wheelchair Joystick flag
    bool wc_js_flag;
    nhp.getParam("wheelchair_joystick", wc_js_flag);

    // Construct the robot structure
    ros::NodeHandle drive_nh("drive_hw");
    drive_hw::DriveHW robot(drive_nh, sim_flag, wc_js_flag);

    // Construct the controller manager
    ros::NodeHandle nh;
    controller_manager::ControllerManager manager(&robot, nh);

    // Spinner
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // LOOP
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(200);
    
    while(ros::ok()) {

        // Get the time
        ros::Time now = ros::Time::now();
        ros::Duration period = now - prev_time;
        prev_time = now;

        // Read, Update, Write the state
        robot.read(now, period);
        manager.update(now, period);
        robot.write(now, period);

        rate.sleep();
    }

    return 0;
}
