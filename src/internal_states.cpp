/**
 * Keeps track of the motor positions of the robot, as broadcasted by raspberry pi.
 * Sends it to the controller node as a bundled message
 * Written by Anmol Kathail (anmolk@seas.upenn.edu)
 **/

#include "internal_states/internal_states.h"

internal_states::internal_states(){
    is = ros::NodeHandle();

    motor_pos_subs.resize(4); //4 motors

    for(unsigned short i = 0; i < motor_pos_subs.size(); i++){ //TODO: Sometimes one motor gets updated multiple times in a single loop? Check.
        sprintf(motor_topic_name, "motor_%02d_", i); //Create a string for the motor name. Starts from Left front (0) and cycles CCW

        motor_pos_subs[i] = is.subscribe<std_msgs::Float64>(motor_topic_name, 1, boost::bind(&internal_states::update_motor_position, this, _1, i)); //Updates each motor position and publishes it
    }

    internal_states_data_out = is.advertise<hqrhex_control::internal_states_msg>("internal_states_data_out", 1); //TODO: messages are not building error
}

//Update motor positions, send it to controller node
void internal_states::update_motor_position(const std_msgs::Float64::ConstPtr &pose_msg, int i){
    switch(i){
        case 0: dataout_msg.LF_motor_pos = pose_msg->data;
            break;
        case 1: dataout_msg.RF_motor_pos = pose_msg->data;
            break;
        case 2: dataout_msg.RR_motor_pos = pose_msg->data;
            break;
        case 3: dataout_msg.LR_motor_pos = pose_msg->data;
            break;
        default: ROS_ERROR("iterator more than number of motors");
            break;
    }

    internal_states_data_out.publish(dataout_msg);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "internal_states");
    ros::NodeHandle is;
    internal_states class_init;
    ros::spin();
    return 0;
}