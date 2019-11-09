/** Finds the CoM of each obstacle in the world space and assigns it an array
Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#pragma once

float log_positions[15];

ros::Subscriber log01_sub;
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

void log01_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[0] = msg->pose.position.x;
}

void log02_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[1] = msg->pose.position.x;
}

void log03_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[2] = msg->pose.position.x;
}

void log04_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[3] = msg->pose.position.x;
}

void log05_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[4] = msg->pose.position.x;
}

void log06_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[5] = msg->pose.position.x;
}

void log07_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[6] = msg->pose.position.x;
}

void log08_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[7] = msg->pose.position.x;
}

void log09_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[8] = msg->pose.position.x;
}

void log10_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[9] = msg->pose.position.x;
}

void log11_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[10] = msg->pose.position.x;
}

void log12_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[11] = msg->pose.position.x;
}

void log13_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[12] = msg->pose.position.x;
}

void log14_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[13] = msg->pose.position.x;
}

void log15_pos(const geometry_msgs::PoseStamped::ConstPtr &msg){
    log_positions[14] = msg->pose.position.x;
}
