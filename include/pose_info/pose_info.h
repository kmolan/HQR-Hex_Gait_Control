/** Calculates the true CoM of the robot body and returns the closest
  * Obstacle to each leg, plus the overall trajectory.
  * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>

#define Pi 3.142859

class pose_info{

public:

    /*!
     * @brief default constructor
     */
    pose_info();

    /*!
     * @brief subscribes to the pose of the robot object of the vicon body, also calls the rest of the functions within this class. Calls all the class publishers
     * @param pose_msg [in] a const pointer to the pose of the robot body
     */
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);

    /*!
     * Updates state value messages before publishing them on the topics
     */
    void updatemessages();

    /*!
     * @brief calls all the publishers in the class
     */
    void publisher_callback();

    /*!
     * @brief prints on console for debugging
     */
    void debug();

private:

        /*!
     * @brief converts the incoming quaternion angles and returns corresponding euler angles via reference
     * @param msg [in] A struct with x,y,z,w components representing the quaternions
     * @param add_roll [out] The roll angle of the body
     * @param add_pitch [out] The pitch angle of the body
     * @param add_yaw [out] The yaw angle of the body
     */
    static void quat_2_euler(geometry_msgs::Quaternion msg, double& add_roll, double& add_pitch, double& add_yaw);

    /*!
     * @brief calculates the 3-D transformation matrix (tm), where tm = yaw*pitch*roll
     * @param roll [in] the roll angle of the body
     * @param pitch [in] the pitch angle of the body
     * @param yaw [in] the yaw angle of the body
     * @param transformation_matrix [out] the returned transformation matrix
     */
    static void transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f& transformation_matrix);

    /*!
     * @brief Converts the observed CoM from the motion capture system to the true CoM of the robot body
     * @param x [in] x-coordinate of the observed CoM
     * @param y [in] y-coordinate of the observed CoM
     * @param z [in] z-coordinate of the observed CoM
     * @param transformation_matrix [in] the rotation matrix from the observed CoM orientation
     * @param add_com_tf [out] The true CoM (x,y,z)  coordinates returned via reference
     */
    static void getCoM(double x, double y, double z, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::Pose& add_com_tf);

    /*!
     * @brief returns the positions (x,y,z) of the 4 hips in world space
     * @param com_tf [in] the position (x,y,z) of the robot CoM
     * @param transformation_matrix [in] the rotation matrix of the robot body (yaw*pitch*roll)
     * @param add_LF_Hip [out] The position of the Left Front hip returned via reference
     * @param add_RF_Hip [out] The position of the Right Front hip returned via reference
     * @param add_RR_Hip [out] The position of the Right Rear hip returned via reference
     * @param add_LR_Hip [out] The position of the Left Rear hip returned via reference
     */
    void getHips(const geometry_msgs::Pose& com_tf, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::Pose& add_LF_Hip, geometry_msgs::Pose& add_RF_Hip, geometry_msgs::Pose& add_RR_Hip, geometry_msgs::Pose& add_LR_Hip);

    /*!
     * @brief Fills the class member array "log_positions" with the position of each obstacle position in x co-ordinate
     * @param msg [in] Pose of obstacle 'i'
     * @param i [in] Index of the obstacle being subscribed to (0-indexed)
     */
    void find_log_positions(const geometry_msgs::PoseStamped::ConstPtr &msg, int i);

    /*!
     * @brief returns the closest obstacle (1-indexed) to each hip, along with the closest distance.
     * @brief The logs are all placed parallel to the x-axis, so smallest distance for a point would always be
     * @brief along the x-direction, regardless of the orientation
     * @param LF_x [in] Left Front Hip's x-coordinate
     * @param RF_x [in] Right Front Hip's x-coordinate
     * @param RR_x [in] Right Rear Hip's x-coordinate
     * @param LR_x [in] Left Rear Hip's x-coordinate
     * @param LF_obs [out] index of the obstacle closest to Left Front Hip
     * @param RF_obs [out] index of the obstacle closest to Right Front Hip
     * @param RR_obs [out] index of the obstacle closest to Right Rear Hip
     * @param LR_obs [out] index of the obstacle closest to Left Rear Hip
     * @param LF_obs_dist [out] distance from the obstacle closest to Left Front Hip
     * @param RF_obs_dist [out] distance from the obstacle closest to Right Front Hip
     * @param RR_obs_dist [out] distance from the obstacle closest to Right Rear Hip
     * @param LR_obs_dist [out] distance from the obstacle closest to Left Rear Hip
     */
    void closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int& LF_obs, int& RF_obs, int& RR_obs, int& LR_obs, double& LF_obs_dist, double& RF_obs_dist, double& RR_obs_dist, double& LR_obs_dist);

    //Physical parameters of the robot
    double robot_length; ///<Length of the robot body in metres
    double robot_width; ///<Width of the robot body in metres
    double hip_offset; ///<HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

    ros::NodeHandle n_; ///<Initializes the node handle

    std::string robot_obj_name;  ///<Name of the robot object created in vicon

    ros::Subscriber pose_sub; ///<Subscribes to the robot CoM

    std::vector<ros::Subscriber> log_subs; ///<Vector of subscribers for each obstacle (15 total)
    char obs_name[40]; ///<Topic name for the obstacle's body

    double log_positions[15]; ///<Array to be filled with position of each obstacle

    ros::Publisher actual_pose; ///<Publishes actual com of the body
    ros::Publisher LF_Hip_pub; ///<publishes left front hip position
    ros::Publisher RF_Hip_pub; ///<publishes right front hip position
    ros::Publisher RR_Hip_pub; ///<publishes right rear hip position
    ros::Publisher LR_Hip_pub; ///<publishes left rear position

    ros::Publisher vicon_data_out_hips; ///< Array publisher containing the positions of the 4 hips
    ros::Publisher vicon_data_out_obs; ///< Array publisher containing the positions of the obstacle positions closest to the hips

    int LF_obs_index; ///<index for the obstacle closest to Left Front Hip
    int RF_obs_index; ///<index for the obstacle closest to Right Front Hip
    int RR_obs_index; ///<index for the obstacle closest to Right Rear Hip
    int LR_obs_index; ///<index for the obstacle closest to Left Rear Hip
    double LF_obs_distance; ///<distance from the obstacle closest to Left Front Hip
    double RF_obs_distance; ///<distance from the obstacle closest to Right Front Hip
    double RR_obs_distance; ///<distance from the obstacle closest to Right Rear Hip
    double LR_obs_distance; ///<distance from the obstacle closest to Left Rear Hip
    int numberOfObstacles; ///<number of obstacles in the workspace

    geometry_msgs::Pose LF_Hip, RF_Hip, RR_Hip, LR_Hip;
    geometry_msgs::PoseArray HipPoseArray; ///< 1-D Array container for hip poses, where index 0 is left front, and assigment is CCW
    geometry_msgs::Pose robot_body_com; ///<pose of the robot CoM
    Eigen::Matrix4f transformation_matrix; ///<transformation matrix from 7-body to 4-body
    double roll; ///<Roll angle to the robot body
    double pitch; ///<pitch angle of the robot body
    double yaw; ///<yaw angle of the robot body
    std_msgs::Float64MultiArray closestObstacleArray; ///< Array with position of closest obstacles, index matches the hip for HipPoseArray
};

/*!
 * @brief Helper function that returns the euclidean distance between two points
 * @param obs [in] position of the obstacle
 * @param logpos [in] position of the log
 * @return euclidean distance
 */
float dist(double obs, double logpos){
    return std::pow(obs - logpos, 2);
}