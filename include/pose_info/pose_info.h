/** Calculates the true CoM of the robot body and returns the closest
 * Obstacle to each leg, plus the overall trajectory.
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

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

/* custom msg type:
 * float64 LF_obs_pos
 * float64 RF_obs_pos
 * float64 RR_obs_pos
 * float64 LR_obs_pos
 * geometry_msgs/PoseStamped LF_Hip_pose
 * geometry_msgs/PoseStamped RF_Hip_pose
 * geometry_msgs/PoseStamped RR_Hip_pose
 * geometry_msgs/PoseStamped LR_Hip_pose */
#include "hqrhex_control/pose_info_msg.h"

#define Pi 3.142859

class pose_info{

public:

    /*!
     * @brief default constructor
     */
    pose_info();

    /*!
     * @brief subscribes to the pose of the robot object of the vicon body, also calls the rest of the functions within this class. Calls all the class publishers
     * @param pose_msg a const pointer to the pose of the robot body
     */
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

    /*!
     * @brief converts the incoming quaternion angles and returns corresponding euler angles via reference
     * @param msg A struct with x,y,z,w components representing the quaternions
     * @param add_roll The roll angle of the body
     * @param add_pitch The pitch angle of the body
     * @param add_yaw The yaw angle of the body
     */
    static void quat_2_euler(geometry_msgs::Quaternion msg, double* add_roll, double* add_pitch, double* add_yaw);

    /*!
     * @brief calculates the 3-D transformation matrix (tm), where tm = yaw*pitch*roll
     * @param roll the roll angle of the body
     * @param pitch the pitch angle of the body
     * @param yaw the yaw angle of the body
     * @param transformation_matrix the returned transformation matrix
     */
    static void transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f* transformation_matrix);

    /*!
     * @brief Converts the observed CoM from the motion capture system to the true CoM of the robot body
     * @param x x-coordinate of the observed CoM
     * @param y y-coordinate of the observed CoM
     * @param z z-coordinate of the observed CoM
     * @param transformation_matrix  the rotation matrix from the observed CoM orientation
     * @param add_com_tf The true CoM (x,y,z)  coordinates returned via reference
     */
    static void getCoM(double x, double y, double z, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped* add_com_tf);

    /*!
     * @brief returns the positions (x,y,z) of the 4 hips in world space
     * @param com_tf the position (x,y,z) of the robot CoM
     * @param transformation_matrix the rotation matrix of the robot body (yaw*pitch*roll)
     * @param add_LF_Hip The position of the Left Front hip returned via reference
     * @param add_RF_Hip The position of the Right Front hip returned via reference
     * @param add_RR_Hip The position of the Right Rear hip returned via reference
     * @param add_LR_Hip The position of the Left Rear hip returned via reference
     */
    void getHips(const geometry_msgs::PoseStamped& com_tf, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped* add_LF_Hip, geometry_msgs::PoseStamped* add_RF_Hip, geometry_msgs::PoseStamped* add_RR_Hip, geometry_msgs::PoseStamped* add_LR_Hip);

    /*!
     * @brief The logs are all parallel to each other, kept along the y-axis. Shortest distance to any log is the x-coordinate distance of the point to the log
     * @brief Returns the index (1-indexed) of the closest obstacle to each hip
     * @param LF_x the x-coordinate of the Left Front Hip to the logs
     * @param RF_x the x-coordinate of the Right Front Hip to the logs
     * @param RR_x the x-coordinate of the Right Rear Hip to the logs
     * @param LR_x the x-coordinate of the Left Rear Hip to the logs
     * @param LF_obs the index of the obstacle closest to Left Front hip, returned via reference
     * @param RF_obs the index of the obstacle closest to Right Front hip, returned via reference
     * @param RR_obs the index of the obstacle closest to Right Rear hip, returned via reference
     * @param LR_obs the index of the obstacle closest to Left Rear hip, returned via reference
     */
    static void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int* LF_obs, int* RF_obs, int* RR_obs, int* LR_obs);

private:

    //Physical parameters of the robot
    double robot_length; //Length of the robot body in metres
    double robot_width; //Width of the robot body in metres
    double hip_offset; //HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

    //ROS stuff
    ros::NodeHandle n_; //Initializes the node handle

    std::string robot_obj_name;  //Name of the robot object created in vicon

    std::string log_01_name; //Names of the log objects in vicon (15 in number)
    std::string log_02_name;
    std::string log_03_name;
    std::string log_04_name;
    std::string log_05_name;
    std::string log_06_name;
    std::string log_07_name;
    std::string log_08_name;
    std::string log_09_name;
    std::string log_10_name;
    std::string log_11_name;
    std::string log_12_name;
    std::string log_13_name;
    std::string log_14_name;
    std::string log_15_name;

    ros::Subscriber pose_sub; //Subscribes to the robot CoM

    ros::Subscriber log01_sub; //one subscriber for each log (15 in total)
    ros::Subscriber log02_sub;
    ros::Subscriber log03_sub;
    ros::Subscriber log04_sub;
    ros::Subscriber log05_sub;
    ros::Subscriber log06_sub;
    ros::Subscriber log07_sub;
    ros::Subscriber log08_sub;
    ros::Subscriber log09_sub;
    ros::Subscriber log10_sub;
    ros::Subscriber log11_sub;
    ros::Subscriber log12_sub;
    ros::Subscriber log13_sub;
    ros::Subscriber log14_sub;
    ros::Subscriber log15_sub;

    ros::Publisher actual_pose; //Publishes actual com of the body
    ros::Publisher yaw_angle; //publishes yaw angle of the robot com
    ros::Publisher pitch_angle; //publishes pitch angle of the robot com
    ros::Publisher LF_Hip_pub; //publishes left front hip position
    ros::Publisher RF_Hip_pub; //publishes right front hip position
    ros::Publisher RR_Hip_pub; //publishes right rear hip position
    ros::Publisher LR_Hip_pub; //publishes left rear position

    ros::Publisher msg_out; //Bundled msg containing the positions of the 4 hips, with the position of the log closest to it
};

/*!
 * @brief Helper function that returns the euclidean distance between two points
 * @param obs position of the obstacle
 * @param logpos positon of the log
 * @return euclidean distance
 */
float dist(double obs, double logpos){
    return std::pow(obs - logpos, 2);
}