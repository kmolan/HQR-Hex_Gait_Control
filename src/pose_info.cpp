/** Master node of the package.
Only contains initializations and some early definitions.
Consult header files for specific functions and implementations.
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

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
#define robot_length 0.48 //Length of the robot body in metres
#define robot_width 0.29 //Width of the robot body in metres
#define hip_offset 0.05 //HQR_Hex has the leg hip not directly attached the the body, but at a vertical offset of 0.05m

#include "quat_2_euler.h" //Converts quaternions to euler angles
#include "transform.h" //Forms the translation and rotation matrices
#include "com.h" //Calculates true CoM of the vicon body
#include "hip_positions.h" //Computes the hip positions of each leg
#include "log_positions.h" //Gets the position of each log in the world frame
#include "closestobstacle.h" //Gets the com of the closest obstacle to each leg hip

std::string robot_obj_name = "/vicon/Yongxin_7Markers/pose"; //Name of the robot object created in vicon
std::string log_01_name = "/vicon/LOG_01_10292019/pose"; //Names of the log objects in vicon
std::string log_02_name = "/vicon/LOG_02_10292019/pose";
std::string log_03_name = "/vicon/LOG_03_10292019/pose";
std::string log_04_name = "/vicon/LOG_04_10292019/pose";
std::string log_05_name = "/vicon/LOG_05_10292019/pose";
std::string log_06_name = "/vicon/LOG_06_10292019/pose";
std::string log_07_name = "/vicon/LOG_07_10292019/pose";
std::string log_08_name = "/vicon/LOG_08_10292019/pose";
std::string log_09_name = "/vicon/LOG_09_10292019/pose";
std::string log_10_name = "/vicon/LOG_10_10292019/pose";
std::string log_11_name = "/vicon/LOG_11_10292019/pose";
std::string log_12_name = "/vicon/LOG_12_10292019/pose";
std::string log_13_name = "/vicon/LOG_13_10292019/pose";
std::string log_14_name = "/vicon/LOG_14_10292019/pose";
std::string log_15_name = "/vicon/LOG_15_10292019/pose";

class testnode {

private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;

public:
    testnode() {
        n = ros::NodeHandle();
        pose_sub = n.subscribe(robot_obj_name, 1, &testnode::pose_callback, this); //Gets the position and orientation of body in vicon space

        log01_sub = n.subscribe(log_01_name, 1 , log01_pos);
        log02_sub = n.subscribe(log_02_name, 1 , log02_pos);
        log03_sub = n.subscribe(log_03_name, 1 , log03_pos);
        log04_sub = n.subscribe(log_04_name, 1 , log04_pos);
        log05_sub = n.subscribe(log_05_name, 1 , log05_pos);
        log06_sub = n.subscribe(log_06_name, 1 , log06_pos);
        log07_sub = n.subscribe(log_07_name, 1 , log07_pos);
        log08_sub = n.subscribe(log_08_name, 1 , log08_pos);
        log09_sub = n.subscribe(log_09_name, 1 , log09_pos);
        log10_sub = n.subscribe(log_10_name, 1 , log10_pos);
        log11_sub = n.subscribe(log_11_name, 1 , log11_pos);
        log12_sub = n.subscribe(log_12_name, 1 , log12_pos);
        log13_sub = n.subscribe(log_13_name, 1 , log13_pos);
        log14_sub = n.subscribe(log_14_name, 1 , log14_pos);
        log15_sub = n.subscribe(log_15_name, 1 , log15_pos);

        actual_pose =  n.advertise<geometry_msgs::PoseStamped>("Body_pose", 1); //publishes the true CoM of the body
        yaw_angle =  n.advertise<std_msgs::Float64>("Body_yaw", 1); //Converted yaw from the quarternion
        pitch_angle = n.advertise<std_msgs::Float64>("Body_pitch", 1); //Converted pitch from the quarternion

        LF_Hip_pub =  n.advertise<geometry_msgs::PoseStamped>("LF_Hip_pose", 1); //publishes the position of the LF Hip
        RF_Hip_pub =  n.advertise<geometry_msgs::PoseStamped>("RF_Hip_pose", 1); //publishes the position of the RF Hip
        RR_Hip_pub =  n.advertise<geometry_msgs::PoseStamped>("RR_Hip_pose", 1); //publishes the position of the RR Hip
        LR_Hip_pub =  n.advertise<geometry_msgs::PoseStamped>("LR_Hip_pose", 1); //publishes the position of the LR Hip
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

        double currentX = pose_msg->pose.position.x; //robot pose X
        double currentY = pose_msg->pose.position.y; //robot pose Y
        double currentZ = pose_msg->pose.position.z; //robot pose z

        double roll, pitch, yaw;

        quat_2_euler(pose_msg->pose.orientation, &roll, &pitch, &yaw); //Convert incoming quaternions to euler angles
        yaw = Pi + yaw; //x axis is pointing opposite to motion, change depending on vicon calibration axes

        Eigen::Matrix4f transformation_matrix;

        transformation_matrices(roll, pitch, yaw, &transformation_matrix); //Returns the transformation matrix for roll (translation not included)

        geometry_msgs::PoseStamped com_tf;

        getCoM(currentX, currentY, currentZ, transformation_matrix, &com_tf); //transform vicon model CoM to true body CoM

        com_tf.pose.orientation = pose_msg->pose.orientation; //CoM orientation is same as vicon model

        geometry_msgs::PoseStamped LF_Hip, RF_Hip, RR_Hip, LR_Hip;

        getHips(com_tf, transformation_matrix, &LF_Hip, &RF_Hip, &RR_Hip, &LR_Hip); //Gets the hip coordinates for all 4 legs

        int LF_obs;
        int RF_obs;
        int RR_obs;
        int LR_obs;

        closestObstacle(LF_Hip.pose.position.x, RF_Hip.pose.position.x, RR_Hip.pose.position.x, LR_Hip.pose.position.x, &LF_obs, &RF_obs, &RR_obs, &LR_obs); //Returns the closest obstacle to each hip

       std::cout<< "LF: " << LF_obs << std::endl;
       std::cout<< "RF: " << RF_obs << std::endl;
       std::cout<< "RR: " << RR_obs << std::endl;
       std::cout<< "LR: " << LR_obs << std::endl;

        //IMP: Don't convert before obtaining all the other necessary geometric parameters since they need radians only.
        yaw = 180*yaw/Pi; //radians to degrees
        pitch = 180*pitch/Pi; //radians to degree, and change sign so pointing up is positive

        std_msgs::Float64 yaw_msg;
        std_msgs::Float64 pitch_msg;

        yaw_msg.data = yaw;
        pitch_msg.data = pitch;

        yaw_angle.publish(yaw_msg);
        pitch_angle.publish(pitch_msg);

//        std::cout<< "Yaw is " << yaw << std::endl; //x axis is from log 1 to log 15, such that log 1 is most negative
//        std::cout<< "Pitch is " << pitch << std::endl;

        actual_pose.publish(com_tf);

        LF_Hip_pub.publish(LF_Hip);
        RF_Hip_pub.publish(RF_Hip);
        RR_Hip_pub.publish(RR_Hip);
        LR_Hip_pub.publish(LR_Hip);

    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    testnode pp;

    ros::spin();
    return 0;
}
