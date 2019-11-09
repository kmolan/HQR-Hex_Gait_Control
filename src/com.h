/** Converts the observed CoM from the vicon data to the true CoM of the robot body
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

ros::Publisher yaw_angle;
ros::Publisher pitch_angle;

void getCoM(double x, double y, double z, Eigen::Matrix4f transformation_matrix, geometry_msgs::PoseStamped* add_com_tf){

    Eigen::MatrixXf temp_pos(4,1);
    Eigen::Matrix4f translation_matrix;
    Eigen::Matrix4f translation_to_origin;
    Eigen::Matrix4f translation_back;
    translation_matrix << 1, 0, 0, -0.0211, 0, 1, 0 ,-0.00293, 0, 0, 1, -0.04238, 0, 0, 0, 1;
    translation_to_origin << 1, 0, 0, -x, 0, 1, 0 ,-y, 0, 0, 1, -z, 0, 0, 0, 1;
    translation_back << 1, 0, 0, x, 0, 1, 0 ,y, 0, 0, 1, z, 0, 0, 0, 1;

    temp_pos << x, y, z, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*translation_matrix*temp_pos;

    add_com_tf->pose.position.x = temp_pos(0,0);
    add_com_tf->pose.position.y = temp_pos(1,0);
    add_com_tf->pose.position.z = temp_pos(2,0);

}
