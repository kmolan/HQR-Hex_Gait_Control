/**
 * Keeps track of the motor positions of the robot, as broadcasted by raspberry pi.
 * Sends it to the controller node as a bundled message
 * Written by Anmol Kathail (anmolk@seas.upenn.edu)
 **/

#include "internal_states/internal_states.h"

internal_states::internal_states(){
    is = ros::NodeHandle();

    motor_pos_subs.resize(4); //4 motors
    dataout_msg.data.resize(4);

    for(unsigned short i = 0; i < motor_pos_subs.size(); i++){ //TODO: Sometimes one motor gets updated multiple times in a single loop? Check.
        sprintf(motor_topic_name, "motor_%02d_", i); //Create a string for the motor name. Starts from Left front (0) and cycles CCW

        motor_pos_subs[i] = is.subscribe<std_msgs::Float64>(motor_topic_name, 1, boost::bind(&internal_states::update_motor_position, this, _1, i)); //Updates each motor position and publishes it
    }

    internal_states_data_out = is.advertise<std_msgs::Float64MultiArray>("internal_states_data_out", 1); //send bundled message towards controller node
}

//Update motor positions and motor frequency
void internal_states::update_motor_position(const std_msgs::Float64::ConstPtr &pose_msg, int i){

    if(!pose_msg){
        ROS_ERROR("In file internal_states, pose pointer is null for iterator %f", double(i));
    }

    dataout_msg.data[i] = pose_msg->data;

    debug();
}

void internal_states::publisher_callback() {
    internal_states_data_out.publish(dataout_msg);
}

void internal_states::debug(){

}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "internal_states");
    ros::NodeHandle is;
    internal_states class_init;
    ros::spin();
    return 0;
}