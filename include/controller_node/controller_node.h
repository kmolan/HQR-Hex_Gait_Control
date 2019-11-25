/** Collects the vicon world data and internal states data from appropriate nodes
  * Implements robot behavior based on desired trajectory
  * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <complex>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "hqrhex_control/internal_states_msg.h"
#include "hqrhex_control/pose_info_msg.h"

class controller_node{
public:

    /*!
     * @brief default constructor
     */
    controller_node();

    /*!
     * @brief subscribes to the motion capture workspace data and updates member variables accordingly
     * @param msg container for all the incoming data, contains position of each closest obstacle (double),
     * pose of each leg hip (gemoetry_msgs::PoseStamped), and body yaw angle (double)
     */
    void vicon_callback(const hqrhex_control::pose_info_msg &msg);

    /*!
     * @brief subscribes to the internal robot states as broadcasted by raspberry pi, updates member variables accordingly
     * @param msg container for position for each of the leg motor positions (double)
     */
    void internal_states_callback(const hqrhex_control::internal_states_msg &msg);

    /*!
     * @brief Updates the contact angle of each hip based on kinematic parameters, calls getTheta()
     */
    void CalculateContactAngles();

    /*!
     * @brief calculates the current angle of trajectory (yaw) based on change over time interval, updates member variables
     */
    void CalculateTrajectoryAngle();

    /*!
     * @brief predicts contact angle of given leg, returns -1 if contact infeasible
     * @param motor_position position of the motor
     * @param Hip_position position of the leg hip
     * @param obs_position position of the nearest obstacle
     * @return predicted contact angle of the leg
     */
    double getTheta(std_msgs::Float64 motor_position, const geometry_msgs::PoseStamped& Hip_position, std_msgs::Float64 obs_position);

    /*!
     * @brief activates suitable legs to turns left, publishes bool flags subscribed by raspberry pi
     */
    void implementTurnLeft();

    /*!
     * @brief activates suitable legs to turns right, publishes bool flags subscribed by raspberry pi
     */
    void implementTurnRight();

    /*!
     * @brief activates suitable legs for least amount of turn, publishes bool flags subscribed by raspberry pi
     */
    void implementTurnZero();

    /*!
     * @brief helper function to calculate L2 norm of two variables
     * @param a first variable
     * @param b second variable
     * @return L2 norm of a 2-D vector with (a,b) coordinates
     */
    static double eucDist(double a, double b);

private:

    ros::NodeHandle cn; //NodeHandle

    ros::Publisher LF_Activation_Flag; //Tells raspberry pi to activate left front leg if true
    ros::Publisher RF_Activation_Flag; //Tells raspberry pi to activate right front leg if true
    ros::Publisher RR_Activation_Flag; //Tells raspberry pi to activate right rear leg if true
    ros::Publisher LR_Activation_Flag; //Tells raspberry pi to activate left rear leg if true

    ros::Subscriber vicon_data_subs; //subscribes to motion capture workspace data
    ros::Subscriber internal_data_subs; //subcribes to robot's internal states data

    double leg_radius; //robot leg radius
    double obstacle_radius; //obstacle radius

    double LF_obs_pos; //position of the obstacle closest to each hip (4) //TODO: create vectors instead
    double RF_obs_pos;
    double RR_obs_pos;
    double LR_obs_pos;

    double body_yaw_angle; //yaw angle of the robot CoM

    geometry_msgs::PoseStamped LF_Hip_pose; //pose of each leg hip (4) //TODO: create vectors instead
    geometry_msgs::PoseStamped RF_Hip_pose;
    geometry_msgs::PoseStamped RR_Hip_pose;
    geometry_msgs::PoseStamped LR_Hip_pose;

    double LF_motor_pos; //motor position of each leg hip (4) //TODO: create vectors instead
    double RF_motor_pos;
    double RR_motor_pos;
    double LR_motor_pos;

    double theta[4]; //predicted obstacle contact angle for each leg, -1 if infeasible

    double traj_loop_enter; //current time
    double traj_loop_exit; //last time CalculateTrajectoryAngle() ran
    double last_trajectory_angle; //last observed yaw angle when CalculateTrajectoryAngle() ran
    double trajectory_loop_time; //duration to check if trajectory has stabilized
    double trajectory_threshold; //threshold to check for yaw angle stability

    double desired_yaw_angle; //desired steady yaw angle of the robot CoM
    double trajectory_angle; //current steady yaw angle of the robot CoM

};