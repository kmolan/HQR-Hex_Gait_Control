/** Computes the transformation matrix using rotation and translation in 3D workspace
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

void transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f* transformation_matrix){

    Eigen::Matrix4f roll_tf;
    Eigen::Matrix4f pitch_tf;
    Eigen::Matrix4f yaw_tf;
    Eigen::Matrix4f translate_tf;

    roll_tf << 1, 0, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, 0, std::sin(roll), std::cos(roll), 0, 0, 0, 0, 1;
    pitch_tf << std::cos(pitch), 0, std::sin(pitch), 0, 0, 1, 0, 0, -std::sin(pitch), 0, std::cos(pitch), 0, 0, 0, 0 ,1;
    yaw_tf << std::cos(yaw), -std::sin(yaw), 0, 0, std::sin(yaw), std::cos(yaw), 0 ,0, 0, 0, 1, 0, 0, 0, 0, 1;

    *transformation_matrix = yaw_tf*pitch_tf*roll_tf;
}
