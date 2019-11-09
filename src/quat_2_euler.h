/** Converts the incoming quarternion angles to euler angles (roll, pitch and yaw)
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

ros::Publisher actual_pose;

void quat_2_euler(const geometry_msgs::Quaternion msg, double* add_roll, double* add_pitch, double* add_yaw){

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(*add_roll, *add_pitch, *add_yaw);

}
