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
     * @brief Updates the position of each leg motor, bundles it into a custom msg, and publishes for controller node to handle
     * @param pose_msg leg motor angle
     * @param i iterator (0 is for Right Front hip motor, and cycles CCW)
     */
    void update_motor_position(const std_msgs::Float64::ConstPtr &pose_msg, int i);

private:

    ros::NodeHandle is; //Node Handle initialization

    std::vector<ros::Subscriber> motor_pos_subs; //Multiple Subscribers each calling a different motor positions
    char motor_topic_name[40]; //Name of topic advertised by raspberry pi

    ros::Publisher internal_states_data_out; //Publishes over topic "internal_states_out" so controller node can read

    /* custom msg type, container for the position of 4 motors
    * float64 LF_motor_pos
    * float64 RF_motor_pos
    * float64 RR_motor_pos
    * float64 LR_motor_pos */
    hqrhex_control::internal_states_msg dataout_msg;
};