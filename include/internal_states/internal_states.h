/**
 * Keeps track of the motor positions of the robot, as broadcasted by raspberry pi.
 * Sends it to the controller node as a bundled message
 * Written by Anmol Kathail (anmolk@seas.upenn.edu)
 **/

#pragma once

#include <ros/ros.h>
#include <ros/ros.h>
#include <cmath>
#include <string>
#include <std_msgs/Float64.h>
#include "hqrhex_control/internal_states_msg.h"

class internal_states{
public:
    /*!
     * @brief default constructor
     */
    internal_states();

    /*!
     * @brief subscribes to the position of the Left Front Hip
     * @param pose_msg the const pointer containing pose information
     */
    void LF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);

    /*!
     * @brief subscribes to the position of the Right Front Hip
     * @param pose_msg the const pointer containing pose information
     */
    void RF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);

    /*!
     * @brief subscribes to the position of the Right Rear Hip
     * @param pose_msg the const pointer containing pose information
     */
    void RR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);

    /*!
     * @brief subscribes to the position of the Left Rear Hip
     * @param pose_msg the const pointer containing pose information
     */
    void LR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg);

private:

    ros::NodeHandle is; //Node Handle initialization

    std::string motor_LF_pos_name; //Holds the (4) names of the topic over which motor positions are being advertised
    std::string motor_RF_pos_name;
    std::string motor_RR_pos_name;
    std::string motor_LR_pos_name;

    ros::Subscriber motor_LF_pos_subs; //Collects data from raspberry pi advertising motor positions of robot
    ros::Subscriber motor_RF_pos_subs;
    ros::Subscriber motor_RR_pos_subs;
    ros::Subscriber motor_LR_pos_subs;

    ros::Publisher internal_states_data_out; //Publishes over topic "internal_states_out" so controller node can read

    /* custom msg type, container for the position of 4 motors
    * float64 LF_motor_pos
    * float64 RF_motor_pos
    * float64 RR_motor_pos
    * float64 LR_motor_pos */
    hqrhex_control::internal_states_msg dataout_msg;
};