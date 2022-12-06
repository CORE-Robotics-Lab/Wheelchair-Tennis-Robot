#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <wtr_navigation/WheelInfo.h>
#include <stdlib.h>     

serial::Serial ser;

void send_command(const wtr_navigation::WheelInfo::ConstPtr& velocity_msg) {   
    // Example: 0.45 1.25\n
    std::string msg = std::to_string(-velocity_msg->left) + " " + std::to_string(velocity_msg->right) + '\n';
    ser.write(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teensy_serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // ROS Sub/Pub
    ros::Publisher pos_pub  = nh.advertise<wtr_navigation::WheelInfo>("/wheel_est_pos", 1);
    ros::Publisher vel_pub  = nh.advertise<wtr_navigation::WheelInfo>("/wheel_est_vel", 1);
    ros::Subscriber vel_sub = nh.subscribe("/wheel_cmd_vel", 1, send_command);

    // Open Serial Device
    std::string serial_device;
    int baud_rate;
    nhp.param("serial_device", serial_device, std::string("/dev/teensy_ros"));
    nhp.param("baud_rate", baud_rate, 500000);

    ROS_INFO_STREAM("Attempting to connect to serial port");
    try {
        ser.setPort(serial_device);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(5);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // Listen to device loop
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        if(ser.available()){
            // Msg Example: 0.12 2.3 2.3 3.4\n (P0, P1, V0, V1)
            std::string msg = ser.read(ser.available());
            
            // Break up message
            std::stringstream ss_msg(msg);
            std::vector<float> msg_data;
            while(ss_msg.good()) {
                std::string substr;
                getline(ss_msg, substr, ' ' );
                msg_data.push_back(std::stod(substr));
            }

            // Send Message
            // wtr_navigation::WheelInfo position_msg;
            // position_msg.left = -msg_data[0];
            // position_msg.right = msg_data[1];
            // pos_pub.publish(position_msg);

            // wtr_navigation::WheelInfo velocity_msg;
            // velocity_msg.left = -msg_data[2];
            // velocity_msg.right = msg_data[3];
            // vel_pub.publish(velocity_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}