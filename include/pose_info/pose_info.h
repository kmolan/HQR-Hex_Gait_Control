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

/* custom msg type, container for the pose of 4 leg hips, and the obstacle distances closest to them
 * float64 LF_obs_pos
 * float64 RF_obs_pos
 * float64 RR_obs_pos
 * float64 LR_obs_pos
 * float64 body_yaw_angle
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
     * @brief Fills an array of 15 with the position of each obstacle position in x co-ordinate
     * @param msg Pose of obstacle 'i'
     * @param i Index of the obstacle being subscribed to (0-indexed)
     */
    void find_log_positions(const geometry_msgs::PoseStamped::ConstPtr &msg, int i);

    /*!
     * @brief returns the closest obstacle (1-indexed) to each hip, along with the closest distance.
     * @brief The logs are all placed parallel to the x-axis, so smallest distance for a point would always be
     * @brief along the x-direction, regardless of the orientation
     * @param LF_x Left Front Hip's x-coordinate
     * @param RF_x Right Front Hip's x-coordinate
     * @param RR_x Right Rear Hip's x-coordinate
     * @param LR_x Left Rear Hip's x-coordinate
     * @param LF_obs index of the obstacle closest to Left Front Hip, returned via reference
     * @param RF_obs index of the obstacle closest to Right Front Hip, returned via reference
     * @param RR_obs index of the obstacle closest to Right Rear Hip, returned via reference
     * @param LR_obs index of the obstacle closest to Left Rear Hip, returned via reference
     * @param LF_obs_dist distance from the obstacle closest to Left Front Hip, returned via reference
     * @param RF_obs_dist distance from the obstacle closest to Right Front Hip, returned via reference
     * @param RR_obs_dist distance from the obstacle closest to Right Rear Hip, returned via reference
     * @param LR_obs_dist distance from the obstacle closest to Left Rear Hip, returned via reference
     */
    void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int* LF_obs, int* RF_obs, int* RR_obs, int* LR_obs, double* LF_obs_dist, double* RF_obs_dist, double* RR_obs_dist, double* LR_obs_dist);

private:

    //Physical parameters of the robot
    double robot_length; //Length of the robot body in metres
    double robot_width; //Width of the robot body in metres
    double hip_offset; //HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

    ros::NodeHandle n_; //Initializes the node handle

    std::string robot_obj_name;  //Name of the robot object created in vicon

    ros::Subscriber pose_sub; //Subscribes to the robot CoM

    std::vector<ros::Subscriber> log_subs; //Vector of subscribers for each obstacle (15 total)
    char obs_name[40]; //Topic name for the obstacle's body

    double log_positions[15]; //Array to be filled with position of each obstacle

    ros::Publisher actual_pose; //Publishes actual com of the body
    ros::Publisher yaw_angle; //publishes yaw angle of the robot com
    ros::Publisher pitch_angle; //publishes pitch angle of the robot com
    ros::Publisher LF_Hip_pub; //publishes left front hip position
    ros::Publisher RF_Hip_pub; //publishes right front hip position
    ros::Publisher RR_Hip_pub; //publishes right rear hip position
    ros::Publisher LR_Hip_pub; //publishes left rear position

    ros::Publisher vicon_data_out; //Bundled msg containing the positions of the 4 hips, with the position of the log closest to it

    int LF_obs_index; //index for the obstacle closest to Left Front Hip
    int RF_obs_index; //index for the obstacle closest to Right Front Hip
    int RR_obs_index; //index for the obstacle closest to Right Rear Hip
    int LR_obs_index; //index for the obstacle closest to Left Rear Hip
    double LF_obs_distance; //distance from the obstacle closest to Left Front Hip
    double RF_obs_distance; //distance from the obstacle closest to Right Front Hip
    double RR_obs_distance; //distance from the obstacle closest to Right Rear Hip
    double LR_obs_distance; //distance from the obstacle closest to Left Rear Hip
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