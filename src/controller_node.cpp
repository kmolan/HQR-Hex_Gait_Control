/** Collects the vicon world data and internal states data from appropriate nodes
  * Implements robot behavior based on desired trajectory
  * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

//TODO: finish the node

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

    vicon_data_hips_subs = cn.subscribe("vicon_data_out_hips", 1, &controller_node::vicon_callback_hips, this); //get data from pose_info.cpp about hips
    vicon_data_obs_subs = cn.subscribe("vicon_data_out_obs", 1, &controller_node::vicon_callback_obs, this); //get data from pose_info.cpp about hips

    internal_data_subs = cn.subscribe("internal_states_data_out", 1, &controller_node::internal_states_callback, this); //get data from internal_states.cpp

    //initialize the pose vectors
    hip_poses.resize(4);
    obstacles_pos.resize(4);
    motor_pos.resize(4);
}

//Update member variables
void controller_node::internal_states_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) { //updates slower than vicon_callback, gets shotgun on activating control scheme

    for(auto i = 0; i<msg->data.size(); i++){ //update the motor pose vector
        motor_pos[i] = msg->data[i];
    }

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
void controller_node::vicon_callback_hips(const geometry_msgs::PoseArray::ConstPtr &msg) {

    for(auto i = 0; i < msg->poses.size(); i++){ //update the hip pose vector
        hip_poses[i] = msg->poses[i];
    }

    //Compute current trajectory
    CalculateTrajectoryAngle();
}

void controller_node::vicon_callback_obs(const std_msgs::Float64MultiArray::ConstPtr &msg) {

    for(auto i = 0; i < msg->data.size(); i++){ //update the obstacle pose vector
        obstacles_pos[i] = msg->data[i];
    }

    debug(); //print stuff on console
}


//predict contact angles on obstacle for each hip, calls getTheta recursively
void controller_node::CalculateContactAngles() {

    ///Use theta[]

}

//get trajectory angle: yaw angle
void controller_node::CalculateTrajectoryAngle() {

    traj_loop_enter  = ros::Time::now().toSec(); //Current time

    //Only update every 5.0 seconds, wait for legs to finish slipping on the obstacles
    if(traj_loop_enter - traj_loop_exit > trajectory_loop_time){ //Find better ways than time, make it leg frequency dependent
        if(last_trajectory_angle - trajectory_angle < trajectory_threshold){ //If I stopped changing too much maybe I'm stable
            trajectory_angle = body_yaw_angle;
            ROS_INFO("%f is the new trajectory angle", trajectory_angle);
        }
    }

    last_trajectory_angle = body_yaw_angle; //updates everytime loop is called. Good or bad? hmmmmm

    traj_loop_exit = traj_loop_enter; //last time loop ran

}

//predict contact angles on obstacle for each hip
double controller_node::getTheta(const std_msgs::Float64 motor_position, const geometry_msgs::PoseStamped& Hip_position,
                               std_msgs::Float64 obs_position) {

    if( eucDist(Hip_position.pose.position.x, Hip_position.pose.position.y) > eucDist(leg_radius, obstacle_radius) ){ //Leg is too far to make any contact
        return -1.0;
    }

    return 0.0;

    //TODO: Write logic for calculating theta
}

//get leg pairings for a left turn
void controller_node::implementTurnLeft() {
    bool flag = false;
    ROS_INFO("Turning left");

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

//get leg pairings for a right turn
void controller_node::implementTurnRight() {
    bool flag = false;
    ROS_INFO("Turning right");
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

//get leg pairings for zero turn
void controller_node::implementTurnZero() {
    bool flag = false;
    ROS_INFO("Maintaining zero");
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

//returns L2 norm between two points
double controller_node::eucDist(const double a, const double b) {
    return sqrt(a*a + b*b);
}

void controller_node::debug(){ //Print stuff on console to debug

    for(auto i = 0; i<obstacles_pos.size(); i++){
        std::cout << i << " " << obstacles_pos[i] << std::endl;
    }
}

//Calls all the publishers
void controller_node::publisher_callback() {
    ///Call all the publishers
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle cn;
    controller_node node;

    ros::spin();
    return 0;
}