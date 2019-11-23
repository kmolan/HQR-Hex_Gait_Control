/**
 * Keeps track of the motor positions of the robot, as broadcasted by raspberry pi.
 * Written by Anmol Kathail (anmolk@seas.upenn.edu)
 **/

#include "internal_states/internal_states.h"

internal_states::internal_states(){
    is = ros::NodeHandle();

    is.getParam("motor_LF_pos_name",motor_LF_pos_name);
    is.getParam("motor_RF_pos_name",motor_RF_pos_name);
    is.getParam("motor_RR_pos_name",motor_RR_pos_name);
    is.getParam("motor_LR_pos_name",motor_LR_pos_name);

    motor_LF_pos_subs = is.subscribe(motor_LF_pos_name, 1, &internal_states::LF_pose_callback, this);
    motor_RF_pos_subs = is.subscribe(motor_RF_pos_name, 1, &internal_states::RF_pose_callback, this);
    motor_RR_pos_subs = is.subscribe(motor_RR_pos_name, 1, &internal_states::RR_pose_callback, this);
    motor_LR_pos_subs = is.subscribe(motor_LR_pos_name, 1, &internal_states::LR_pose_callback, this);
}

void internal_states::LF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg) {

    motorpos.motor_LF_pos = pose_msg->data;
}

void internal_states::RF_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg) {

    motorpos.motor_RF_pos = pose_msg->data;
}

void internal_states::RR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg) {

    motorpos.motor_RR_pos = pose_msg->data;
}

void internal_states::LR_pose_callback(const std_msgs::Float64::ConstPtr &pose_msg) {

    motorpos.motor_LR_pos = pose_msg->data;
}

void calculate_contact_pos(){

}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "internal_states");
    ros::NodeHandle is;
    ros::spin();
    return 0;
}