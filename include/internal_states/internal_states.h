/**
 * Keeps track of the motor positions of the robot, as broadcasted by raspberry pi.
 * Written by Anmol Kathail (anmolk@seas.upenn.edu)
 **/

#pragma once

#include <ros/ros.h>
#include <ros/ros.h>
#include <cmath>
#include <string>
#include <std_msgs/Float64.h>

class internal_states{
public:
    internal_states();

    void LF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);
    void RF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);
    void RR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);
    void LR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);

    void calculate_contact_pos();

private:

    struct motor_positions{
        double motor_LF_pos;
        double motor_RF_pos;
        double motor_RR_pos;
        double motor_LR_pos;
    }motorpos;

    ros::NodeHandle is;

    std::string motor_LF_pos_name;
    std::string motor_RF_pos_name;
    std::string motor_RR_pos_name;
    std::string motor_LR_pos_name;

    ros::Subscriber motor_LF_pos_subs;
    ros::Subscriber motor_RF_pos_subs;
    ros::Subscriber motor_RR_pos_subs;
    ros::Subscriber motor_LR_pos_subs;

};