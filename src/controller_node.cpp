/** Collects the vicon world data and internal states data from appropriate nodes
  * Implements robot behavior based on desired trajectory
  * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/


#include "controller_node/controller_node.h"

controller_node::controller_node() {

    cn = ros::NodeHandle();

    cn.getParam("desired_yaw_angle", desired_yaw_angle);
    cn.getParam("leg_radius", leg_radius);
    cn.getParam("obstacle_radius", obstacle_radius);

    cn.getParam("trajectory_loop_time", trajectory_loop_time);
    cn.getParam("trajectory_threshold", trajectory_threshold);

    LF_Activation_Flag = cn.advertise<std_msgs::Bool>("LF_Activation_Flag",1); //talks to raspberry pi
    RF_Activation_Flag = cn.advertise<std_msgs::Bool>("RF_Activation_Flag",1); //talks to raspberry pi
    RR_Activation_Flag = cn.advertise<std_msgs::Bool>("RR_Activation_Flag",1); //talks to raspberry pi
    LR_Activation_Flag = cn.advertise<std_msgs::Bool>("LR_Activation_Flag",1); //talks to raspberry pi

    vicon_data_subs = cn.subscribe("vicon_data_out", 1, &controller_node::vicon_callback, this); //get data from pose_info.cpp

    internal_data_subs = cn.subscribe("internal_states_data_out", 1, &controller_node::internal_states_callback, this); //get data from internal_states.cpp
}

//Update member variables
void controller_node::internal_states_callback(const hqrhex_control::internal_states_msg &msg) { //updates slower than vicon_callback, gets shotgun on activating control scheme

//    LF_motor_pos = msg->LF_motor_pos;
//    RF_motor_pos = msg->RF_motor_pos;
//    RR_motor_pos = msg->RR_motor_pos;
//    LR_motor_pos = msg->LR_motor_pos;

    CalculateContactAngles(); //predict obstacle contact angles for each hip

    //Depending on the case, activate suitable legs
    if(trajectory_angle > desired_yaw_angle){
        implementTurnLeft();
    }
    else if (trajectory_angle < desired_yaw_angle){
        implementTurnRight();
    }
    else{ //TODO: Threshold instead of exactly equal?
        implementTurnZero();
    }
}

//Update member variables
void controller_node::vicon_callback(const hqrhex_control::pose_info_msg &msg) {

//    LF_obs_pos = msg->LF_obs_pos;
//    RF_obs_pos = msg->RF_obs_pos;
//    RR_obs_pos = msg->RR_obs_pos;
//    LR_obs_pos = msg->LR_obs_pos;
//    body_yaw_angle = msg->body_yaw_angle;
//
//    LF_Hip_pose = msg->LF_Hip_pose;
//    RF_Hip_pose = msg->RF_Hip_pose;
//    RR_Hip_pose = msg->RR_Hip_pose;
//    LR_Hip_pose = msg->LR_Hip_pose;

    //Compute current trajectory
    CalculateTrajectoryAngle();
}


//ContactAngles what
void controller_node::CalculateContactAngles() {

//    theta[0] = getTheta(LF_motor_pos, LF_Hip_pose, LF_obs_pos); //TODO: Fix the message generation issue
//    theta[1] = getTheta(RF_motor_pos, RF_Hip_pose, RF_obs_pos); //TODO: Find a prettier way to do this
//    theta[2] = getTheta(RR_motor_pos, RR_Hip_pose, RR_obs_pos);
//    theta[3] = getTheta(LR_motor_pos, LR_Hip_pose, LR_obs_pos);

}

void controller_node::CalculateTrajectoryAngle() {

    traj_loop_enter  = ros::Time::now().toSec(); //Current time

    //Only update every 5.0 seconds, wait for legs to finish slipping on the obstacles
    if(traj_loop_enter - traj_loop_exit > trajectory_loop_time){ //Find better ways than time, make it leg frequency dependent
        if(last_trajectory_angle - trajectory_angle < trajectory_threshold){ //If I stopped changing too much maybe I'm stable
            trajectory_angle = body_yaw_angle;
        }
    }

    last_trajectory_angle = body_yaw_angle; //updates everytime loop is called. Good or bad? hmmmmm

    traj_loop_exit = traj_loop_enter; //last time loop ran

}

double controller_node::getTheta(const std_msgs::Float64 motor_position, const geometry_msgs::PoseStamped& Hip_position,
                               std_msgs::Float64 obs_position) {

    if( eucDist(Hip_position.pose.position.x, Hip_position.pose.position.y) > eucDist(leg_radius, obstacle_radius) ){ //Leg is too far to make any contact
        return -1.0;
    }

    return 0.0;

    ///Write logic for calculating theta
}

void controller_node::implementTurnLeft() {
    bool flag = false;

    /* pseudo-code:
     * if orientation angle less than 45ish:
     *      find LF-RR combo that produces the most left twist depending on predicted theta
     *      find RF-LR combo that produces that most left twist depending on predicted theta
     *
     *      set bool flags appropriately
     *      ros::publisher.publish(flag) //TODO: Calculate latency in master-slave transmission
     * else:
     *      same pairs but twist contribution is different because of the change in signs
     * */
}

void controller_node::implementTurnRight() {
    bool flag = false;

    /* pseudo-code:
     * if orientation angle less than 45ish:
     *      find LF-RR combo that produces the most right twist depending on predicted theta
     *      find RF-LR combo that produces that most right twist depending on predicted theta
     *
     *      set bool flags appropriately
     *      ros::publisher.publish(flag) //TODO: Calculate latency in master-slave transmission
     * else:
     *      same pairs but twist contribution is different because of the change in signs
     * */

}

void controller_node::implementTurnZero() {
    bool flag = false;

    /* pseudo-code:
     * if orientation angle less than 45ish:
     *      find LF-RR combo that produces the least twist depending on predicted theta
     *      find RF-LR combo that produces that least twist depending on predicted theta
     *
     *      set bool flags appropriately
     *      ros::publisher.publish(flag) //TODO: Calculate latency in master-slave transmission
     * else:
     *      same pairs but twist contribution is different because of the change in signs
     * */

}

double controller_node::eucDist(const double a, const double b) {
    return sqrt(a*a + b*b);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle cn;
    controller_node node;

    ros::spin();
    return 0;
}