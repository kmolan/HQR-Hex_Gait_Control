/** Finds the CoM of each leg hip on the robot body
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

ros::Publisher LF_Hip_pub;
ros::Publisher RF_Hip_pub;
ros::Publisher RR_Hip_pub;
ros::Publisher LR_Hip_pub;

void getHips(geometry_msgs::PoseStamped com_tf, Eigen::Matrix4f transformation_matrix, geometry_msgs::PoseStamped* add_LF_Hip, geometry_msgs::PoseStamped* add_RF_Hip, geometry_msgs::PoseStamped* add_RR_Hip, geometry_msgs::PoseStamped* add_LR_Hip){

    Eigen::MatrixXf temp_pos(4,1);
    Eigen::Matrix4f translation_to_origin;
    Eigen::Matrix4f translation_back;
    translation_to_origin << 1, 0, 0, -com_tf.pose.position.x, 0, 1, 0 ,-com_tf.pose.position.y, 0, 0, 1, -com_tf.pose.position.z, 0, 0, 0, 1;
    translation_back << 1, 0, 0, com_tf.pose.position.x, 0, 1, 0 ,com_tf.pose.position.y, 0, 0, 1, com_tf.pose.position.z, 0, 0, 0, 1;

    temp_pos << com_tf.pose.position.x - robot_length/2, com_tf.pose.position.y - robot_width/2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_LF_Hip->pose.position.x = temp_pos(0,0);
    add_LF_Hip->pose.position.y = temp_pos(1,0);
    add_LF_Hip->pose.position.z = temp_pos(2,0);

    add_LF_Hip->pose.orientation = com_tf.pose.orientation;

    temp_pos << com_tf.pose.position.x - robot_length/2, com_tf.pose.position.y + robot_width/2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_RF_Hip->pose.position.x = temp_pos(0,0);
    add_RF_Hip->pose.position.y = temp_pos(1,0);
    add_RF_Hip->pose.position.z = temp_pos(2,0);

    add_RF_Hip->pose.orientation = com_tf.pose.orientation;

    temp_pos << com_tf.pose.position.x + robot_length/2, com_tf.pose.position.y + robot_width/2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_RR_Hip->pose.position.x = temp_pos(0,0);
    add_RR_Hip->pose.position.y = temp_pos(1,0);
    add_RR_Hip->pose.position.z = temp_pos(2,0);

    add_RR_Hip->pose.orientation = com_tf.pose.orientation;

    temp_pos << com_tf.pose.position.x + robot_length/2, com_tf.pose.position.y - robot_width/2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_LR_Hip->pose.position.x = temp_pos(0,0);
    add_LR_Hip->pose.position.y = temp_pos(1,0);
    add_LR_Hip->pose.position.z = temp_pos(2,0);

    add_LR_Hip->pose.orientation = com_tf.pose.orientation;
}
