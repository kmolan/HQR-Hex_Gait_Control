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

#define Pi 3.142859

class pose_info{

public:
    pose_info();

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

    static void quat_2_euler(geometry_msgs::Quaternion msg, double* add_roll, double* add_pitch, double* add_yaw);

    static void transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f* transformation_matrix);

    static void getCoM(double x, double y, double z, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped* add_com_tf);

    void getHips(const geometry_msgs::PoseStamped& com_tf, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped* add_LF_Hip, geometry_msgs::PoseStamped* add_RF_Hip, geometry_msgs::PoseStamped* add_RR_Hip, geometry_msgs::PoseStamped* add_LR_Hip);

    static void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int* LF_obs, int* RF_obs, int* RR_obs, int* LR_obs);

private:

    //Physical parameters of the robot
    double robot_length; //Length of the robot body in metres
    double robot_width; //Width of the robot body in metres
    double hip_offset; //HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

    //ROS stuff
    ros::NodeHandle n_;

    std::string robot_obj_name;  //Name of the robot object created in vicon
    std::string log_01_name; //Names of the log objects in vicon
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

    ros::Subscriber pose_sub; //Subscribes to the robot com
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
    ros::Publisher LF_closest_obs; //publishes the closest obstacle to LF
    ros::Publisher RF_closest_obs; //publishes the closest obstacle to RF
    ros::Publisher RR_closest_obs; //publishes the closest obstacle to RR
    ros::Publisher LR_closest_obs; //publishes the closest obstacle to LR
};

float dist(double obs, double logpos){ //Helper function that returns the euclidean distance between two points
    return std::pow(obs - logpos, 2);
}