/** Calculates the true CoM of the robot body and returns the closest
 * Obstacle to each leg, plus the overall trajectory.
 * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#include "pose_info/pose_info.h"

pose_info::pose_info() {

    n_ = ros::NodeHandle(); //Initialize node handle

    loadParams();

    pose_sub = n_.subscribe(robot_obj_name, 1, &pose_info::pose_callback, this); //Gets the position and orientation of body in vicon space

    //initialize the arrays
    log_subs.resize(numberOfObstacles); //15 obstacles
    HipPoseArray.poses.resize(4);
    closestObstacleArray.data.resize(4);

    for(unsigned short i = 0; i < log_subs.size(); i++){
        sprintf(obs_name, "/vicon/LOG_%02d_10292019/pose", i + 1); //Create a string for the obstacle name

        log_subs[i] = n_.subscribe<geometry_msgs::PoseStamped>(obs_name, 1, boost::bind(&pose_info::find_log_positions, this, _1, i)); //Fill the log_positions array with obstacle positions
    }

    actual_pose =  n_.advertise<geometry_msgs::Pose>("Body_pose", 1); //publishes the true CoM of the body

    LF_Hip_pub =  n_.advertise<geometry_msgs::Pose>("LF_Hip_pose", 1); //publishes the position of the LF Hip
    RF_Hip_pub =  n_.advertise<geometry_msgs::Pose>("RF_Hip_pose", 1); //publishes the position of the RF Hip
    RR_Hip_pub =  n_.advertise<geometry_msgs::Pose>("RR_Hip_pose", 1); //publishes the position of the RR Hip
    LR_Hip_pub =  n_.advertise<geometry_msgs::Pose>("LR_Hip_pose", 1); //publishes the position of the LR Hip

    vicon_data_out_hips = n_.advertise<geometry_msgs::PoseArray>("vicon_data_out_hips", 1); //vector of hip pose array to controller node
    vicon_data_out_obs = n_.advertise<std_msgs::Float64MultiArray>("vicon_data_out_obs", 1); //vector of obstacle pose array to controller node
}

//Load all the parameters
void pose_info::loadParams(){
    n_.getParam("robot_length", robot_length);
    n_.getParam("robot_width", robot_width);
    n_.getParam("hip_offset", hip_offset);
    n_.getParam("robot_name", robot_obj_name);
    n_.getParam("numberOfObstacles", numberOfObstacles);

    if(!n_.getParam("robot_length", robot_length)){
        ROS_ERROR("In file pose_info robot_length is undefined");
    }

    if(!n_.getParam("robot_width", robot_width)){
        ROS_ERROR("In file pose_info robot_width is undefined");
    }

    if(!n_.getParam("hip_offset", hip_offset)){
        ROS_ERROR("In file pose_info hip_offset is undefined");
    }

    if(!n_.getParam("robot_name", robot_obj_name)){
        ROS_ERROR("In file pose_info robot_name is undefined");
    }

    if(!n_.getParam("numberOfObstacles", numberOfObstacles)){
        ROS_ERROR("In file pose_info numberOfObstacles is undefined");
    }
}

void pose_info::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

    double currentX = pose_msg->pose.position.x; //robot pose X
    double currentY = pose_msg->pose.position.y; //robot pose Y
    double currentZ = pose_msg->pose.position.z; //robot pose z

    quat_2_euler(pose_msg->pose.orientation, roll, pitch, yaw); //Convert incoming quaternions to euler angles
    yaw = Pi + yaw; //x axis is pointing opposite to motion, change depending on vicon calibration axes //TODO:check vicon calibration beforehand

    transformation_matrices(roll, pitch, yaw, transformation_matrix); //Returns the transformation matrix for roll (translation not included)

    getCoM(currentX, currentY, currentZ, transformation_matrix, robot_body_com); //transform vicon model CoM to true body CoM

    robot_body_com.orientation = pose_msg->pose.orientation; //CoM orientation is same as vicon model

    getHips(robot_body_com, transformation_matrix, LF_Hip, RF_Hip, RR_Hip, LR_Hip); //Gets the hip coordinates for all 4 legs

    closestObstacle(LF_Hip.position.x, RF_Hip.position.x, RR_Hip.position.x, LR_Hip.position.x, LF_obs_index, RF_obs_index, RR_obs_index, LR_obs_index, LF_obs_distance, RF_obs_distance, RR_obs_distance, LR_obs_distance); //Returns the closest obstacle to each hip

    updatemessages();
    debug();
    publisher_callback();
}

void pose_info::quat_2_euler(const geometry_msgs::Quaternion msg, double& add_roll, double& add_pitch, double& add_yaw) {

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to access roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(add_roll, add_pitch, add_yaw);
}

void pose_info::transformation_matrices(const double roll, const double pitch, const double yaw, Eigen::Matrix4f& transformation_matrix) {

    Eigen::Matrix4f roll_tf;
    Eigen::Matrix4f pitch_tf;
    Eigen::Matrix4f yaw_tf;
    Eigen::Matrix4f translate_tf;

    roll_tf << 1, 0, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, 0, std::sin(roll), std::cos(roll), 0, 0, 0, 0, 1; //roll 3-D matrix
    pitch_tf << std::cos(pitch), 0, std::sin(pitch), 0, 0, 1, 0, 0, -std::sin(pitch), 0, std::cos(pitch), 0, 0, 0, 0 ,1; //pitch 3-D matrix
    yaw_tf << std::cos(yaw), -std::sin(yaw), 0, 0, std::sin(yaw), std::cos(yaw), 0 ,0, 0, 0, 1, 0, 0, 0, 0, 1; //yaw 3-D matrix

    transformation_matrix = yaw_tf*pitch_tf*roll_tf;
}

void pose_info::getCoM(const double x, const double y, const double z, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::Pose& add_com_tf) {

    Eigen::MatrixXf temp_pos(4,1);
    Eigen::Matrix4f translation_matrix;
    Eigen::Matrix4f translation_to_origin;
    Eigen::Matrix4f translation_back;
    translation_matrix << 1, 0, 0, -0.0211, 0, 1, 0 ,-0.00293, 0, 0, 1, -0.04238, 0, 0, 0, 1;
    translation_to_origin << 1, 0, 0, -x, 0, 1, 0 ,-y, 0, 0, 1, -z, 0, 0, 0, 1;
    translation_back << 1, 0, 0, x, 0, 1, 0 ,y, 0, 0, 1, z, 0, 0, 0, 1;

    temp_pos << x, y, z, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*translation_matrix*temp_pos; //true robot CoM

    //return via reference
    add_com_tf.position.x = temp_pos(0,0);
    add_com_tf.position.y = temp_pos(1,0);
    add_com_tf.position.z = temp_pos(2,0);
}

void pose_info::getHips(const geometry_msgs::Pose& com_tf_var, const Eigen::Matrix4f& transformation_matrix_var, geometry_msgs::Pose& add_LF_Hip, geometry_msgs::Pose& add_RF_Hip, geometry_msgs::Pose& add_RR_Hip, geometry_msgs::Pose& add_LR_Hip) {

    Eigen::MatrixXf temp_pos(4,1);
    Eigen::Matrix4f translation_to_origin;
    Eigen::Matrix4f translation_back;
    translation_to_origin << 1, 0, 0, -robot_body_com.position.x, 0, 1, 0 ,-robot_body_com.position.y, 0, 0, 1, -robot_body_com.position.z, 0, 0, 0, 1;
    translation_back << 1, 0, 0, robot_body_com.position.x, 0, 1, 0 ,robot_body_com.position.y, 0, 0, 1, robot_body_com.position.z, 0, 0, 0, 1;

    //LF hip pose
    temp_pos << com_tf_var.position.x - robot_length / 2, com_tf_var.position.y - robot_width / 2, com_tf_var.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix_var*translation_to_origin*temp_pos;

    add_LF_Hip.position.x = temp_pos(0,0);
    add_LF_Hip.position.y = temp_pos(1,0);
    add_LF_Hip.position.z = temp_pos(2,0);

    add_LF_Hip.orientation = com_tf_var.orientation;

    //RF hip pose
    temp_pos << com_tf_var.position.x - robot_length / 2, com_tf_var.position.y + robot_width / 2, com_tf_var.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix_var*translation_to_origin*temp_pos;

    add_RF_Hip.position.x = temp_pos(0,0);
    add_RF_Hip.position.y = temp_pos(1,0);
    add_RF_Hip.position.z = temp_pos(2,0);

    add_RF_Hip.orientation = com_tf_var.orientation;

    //RR hip pose
    temp_pos << com_tf_var.position.x + robot_length / 2, com_tf_var.position.y + robot_width / 2, com_tf_var.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix_var*translation_to_origin*temp_pos;

    add_RR_Hip.position.x = temp_pos(0,0);
    add_RR_Hip.position.y = temp_pos(1,0);
    add_RR_Hip.position.z = temp_pos(2,0);

    add_RR_Hip.orientation = com_tf_var.orientation;

    //LR hip pose
    temp_pos << com_tf_var.position.x + robot_length / 2, com_tf_var.position.y - robot_width / 2, com_tf_var.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix_var*translation_to_origin*temp_pos;

    add_LR_Hip.position.x = temp_pos(0,0);
    add_LR_Hip.position.y = temp_pos(1,0);
    add_LR_Hip.position.z = temp_pos(2,0);

    add_LR_Hip.orientation = com_tf_var.orientation;
}

//Update log positions
void pose_info::find_log_positions(const geometry_msgs::PoseStamped::ConstPtr &msg, int i){

    log_positions[i] = msg->pose.position.x;

    if(!msg){
        ROS_ERROR("In file pose_info, obstacle pointer for iterator %f is null", double(i));
    }
}

//Find closest obstacle to each hip
void pose_info::closestObstacle(const double LF_x, const double RF_x, const double RR_x, const double LR_x, int &LF_obs, int &RF_obs, int &RR_obs, int &LR_obs, double& LF_obs_dist, double& RF_obs_dist, double& RR_obs_dist, double& LR_obs_dist) {

    LF_obs_dist = FLT_MAX;
    RF_obs_dist = FLT_MAX;
    RR_obs_dist = FLT_MAX;
    LR_obs_dist = FLT_MAX;

    for(int i = 0; i < 15; i++){

        if(dist(LF_x,log_positions[i])< LF_obs_dist){
            LF_obs = i+1;
            LF_obs_dist = dist(LF_x,log_positions[i]);
        }

        if(dist(RF_x,log_positions[i]) < RF_obs_dist){
            RF_obs = i+1;
            RF_obs_dist = dist(RF_x,log_positions[i]);
        }

        if(dist(RR_x,log_positions[i]) < RR_obs_dist){
            RR_obs = i+1;
            RR_obs_dist = dist(RR_x,log_positions[i]);
        }

        if(dist(LR_x,log_positions[i]) < LR_obs_dist){
            LR_obs = i+1;
            LR_obs_dist = dist(LR_x,log_positions[i]);
        }
    }
}

void pose_info::updatemessages() { //Update the message types for the publishers

    HipPoseArray.poses[0] = LF_Hip;
    HipPoseArray.poses[1] = RF_Hip;
    HipPoseArray.poses[2] = RR_Hip;
    HipPoseArray.poses[3] = LR_Hip;

    closestObstacleArray.data[0] = LF_obs_distance;
    closestObstacleArray.data[1] = RF_obs_distance;
    closestObstacleArray.data[2] = LR_obs_distance;
    closestObstacleArray.data[3] = RR_obs_distance;
}

void pose_info::debug(){ //print stuff on console to debug. If commented out, it means I'm not debugging that stuff

//    std::cout << "LF: " << LF_obs_index << std::endl;
//    std::cout << "RF: " << RF_obs_index << std::endl;
//    std::cout << "RR: " << RR_obs_index << std::endl;
//    std::cout << "LR: " << LR_obs_index << std::endl;

}

void pose_info::publisher_callback() { //call all the publishers

    actual_pose.publish(robot_body_com);
    LF_Hip_pub.publish(LF_Hip);
    RF_Hip_pub.publish(RF_Hip);
    RR_Hip_pub.publish(RR_Hip);
    LR_Hip_pub.publish(LR_Hip);
    vicon_data_out_hips.publish(HipPoseArray);
    vicon_data_out_obs.publish(closestObstacleArray);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "pose_info");
    ros::NodeHandle n;
    pose_info pi;

    ros::spin();
    return 0;
}
