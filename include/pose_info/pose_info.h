/** Only contains declarations and function headers.
Consult pose_info.cpp for specific implementations.
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <complex>
#include <cmath>

#define Pi 3.142859

class pose_info{

public:
    pose_info();

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

    void quat_2_euler(const geometry_msgs::Quaternion msg, double* add_roll, double* add_pitch, double* add_yaw);

    void transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f* transformation_matrix);

    void getCoM(double x, double y, double z, Eigen::Matrix4f transformation_matrix, geometry_msgs::PoseStamped* add_com_tf);

    void getHips(geometry_msgs::PoseStamped com_tf, Eigen::Matrix4f transformation_matrix, geometry_msgs::PoseStamped* add_LF_Hip, geometry_msgs::PoseStamped* add_RF_Hip, geometry_msgs::PoseStamped* add_RR_Hip, geometry_msgs::PoseStamped* add_LR_Hip);

    void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int* LF_obs, int* RF_obs, int* RR_obs, int* LR_obs);

private:

    //Physical parameters of the robot
    double robot_length_; //Length of the robot body in metres
    double robot_width_; //Width of the robot body in metres
    double hip_offset_; //HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

    //ROS stuff
    ros::NodeHandle n_;

    std::string robot_obj_name_;  //Name of the robot object created in vicon
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


    //Almost all the publishers are mostly for debugging purposes. I was plotting them on PlotJuggler
    ros::Publisher actual_pose_; //Publishes actual com of the body
    ros::Publisher yaw_angle_; //publishes yaw angle of the robot com
    ros::Publisher pitch_angle_; //publishes pitch angle of the robot com
    ros::Publisher LF_Hip_pub_; //publishes left front hip position
    ros::Publisher RF_Hip_pub_; //publishes right front hip position
    ros::Publisher RR_Hip_pub_; //publishes right rear hip position
    ros::Publisher LR_Hip_pub_; //publishes left rear position

};

float dist(double obs, double logpos){ //Helper function that returns the euclidean distance between two points
    return std::pow(obs - logpos, 2);
}