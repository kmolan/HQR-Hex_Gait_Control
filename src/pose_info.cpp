/** Calculates the true CoM of the robot body and returns the closest
 * Obstacle to each leg, plus the overall trajectory.
 * Written by Anmol Kathail (anmolk@seas.upenn.edu) **/

#include "pose_info/pose_info.h"

pose_info::pose_info() {

    n_ = ros::NodeHandle();

    n_.getParam("robot_length", robot_length);
    n_.getParam("robot_width", robot_width);
    n_.getParam("hip_offset", hip_offset);

    n_.getParam("robot_name", robot_obj_name);

    pose_sub = n_.subscribe(robot_obj_name, 1, &pose_info::pose_callback, this); //Gets the position and orientation of body in vicon space

    log_subs.resize(15); //15 obstacles

    for(unsigned short i = 0; i < log_subs.size(); i++){ //TODO: Sometimes one obstacle gets updated multiple times in a single loop? Check.
        sprintf(obs_name, "/vicon/LOG_%02d_10292019/pose", i + 1); //Create a string for the obstacle name

        log_subs[i] = n_.subscribe<geometry_msgs::PoseStamped>(obs_name, 1, boost::bind(&pose_info::find_log_positions, this, _1, i)); //Fill the log_positions array with obstacle positions
    }

    actual_pose =  n_.advertise<geometry_msgs::PoseStamped>("Body_pose", 1); //publishes the true CoM of the body
    yaw_angle =  n_.advertise<std_msgs::Float64>("Body_yaw", 1); //Converted yaw from the quaternion
    pitch_angle = n_.advertise<std_msgs::Float64>("Body_pitch", 1); //Converted pitch from the quaternion

    LF_Hip_pub =  n_.advertise<geometry_msgs::PoseStamped>("LF_Hip_pose", 1); //publishes the position of the LF Hip
    RF_Hip_pub =  n_.advertise<geometry_msgs::PoseStamped>("RF_Hip_pose", 1); //publishes the position of the RF Hip
    RR_Hip_pub =  n_.advertise<geometry_msgs::PoseStamped>("RR_Hip_pose", 1); //publishes the position of the RR Hip
    LR_Hip_pub =  n_.advertise<geometry_msgs::PoseStamped>("LR_Hip_pose", 1); //publishes the position of the LR Hip

    vicon_data_out = n_.advertise<hqrhex_control::pose_info_msg>("vicon_data_out", 1); //TODO: messages are not building error
}

void pose_info::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

    double currentX = pose_msg->pose.position.x; //robot pose X
    double currentY = pose_msg->pose.position.y; //robot pose Y
    double currentZ = pose_msg->pose.position.z; //robot pose z

    double roll, pitch, yaw;

    quat_2_euler(pose_msg->pose.orientation, &roll, &pitch, &yaw); //Convert incoming quaternions to euler angles
    yaw = Pi + yaw; //x axis is pointing opposite to motion, change depending on vicon calibration axes

    Eigen::Matrix4f transformation_matrix;

    transformation_matrices(roll, pitch, yaw, &transformation_matrix); //Returns the transformation matrix for roll (translation not included)

    geometry_msgs::PoseStamped com_tf;

    getCoM(currentX, currentY, currentZ, transformation_matrix, &com_tf); //transform vicon model CoM to true body CoM

    com_tf.pose.orientation = pose_msg->pose.orientation; //CoM orientation is same as vicon model

    geometry_msgs::PoseStamped LF_Hip, RF_Hip, RR_Hip, LR_Hip;

    hqrhex_control::pose_info_msg vicon_data_out_var;

    getHips(com_tf, transformation_matrix, &LF_Hip, &RF_Hip, &RR_Hip, &LR_Hip); //Gets the hip coordinates for all 4 legs

   closestObstacle(LF_Hip.pose.position.x, RF_Hip.pose.position.x, RR_Hip.pose.position.x, LR_Hip.pose.position.x, &LF_obs_index, &RF_obs_index, &RR_obs_index, &LR_obs_index, &LF_obs_distance, &RF_obs_distance, &RR_obs_distance, &LR_obs_distance); //Returns the closest obstacle to each hip

   std::cout << "LF: " << LF_obs_index << std::endl;
   std::cout << "RF: " << RF_obs_index << std::endl;
   std::cout << "RR: " << RR_obs_index << std::endl;
   std::cout << "LR: " << LR_obs_index << std::endl;

    //IMP: Don't convert before obtaining all the other necessary geometric parameters since they need radians only.
    yaw = 180*yaw/Pi; //radians to degrees
    pitch = 180*pitch/Pi; //radians to degree, and change sign so pointing up is positive

    std_msgs::Float64 yaw_msg;
    std_msgs::Float64 pitch_msg;

    yaw_msg.data = yaw;
    pitch_msg.data = pitch;

    yaw_angle.publish(yaw_msg);
    pitch_angle.publish(pitch_msg);

//        std::cout<< "Yaw is " << yaw << std::endl; //x axis is from log 1 to log 15, such that log 1 is most negative
//        std::cout<< "Pitch is " << pitch << std::endl;

    actual_pose.publish(com_tf);

    LF_Hip_pub.publish(LF_Hip);
    RF_Hip_pub.publish(RF_Hip);
    RR_Hip_pub.publish(RR_Hip);
    LR_Hip_pub.publish(LR_Hip);

    vicon_data_out_var.LF_obs_pos = LF_obs_distance;
    vicon_data_out_var.LF_obs_pos = LF_obs_distance;
    vicon_data_out_var.LF_obs_pos = LF_obs_distance;
    vicon_data_out_var.LF_obs_pos = LF_obs_distance;
    vicon_data_out_var.body_yaw_angle = yaw;
    vicon_data_out_var.LF_Hip_pose = LF_Hip;
    vicon_data_out_var.LF_Hip_pose = RF_Hip;
    vicon_data_out_var.LF_Hip_pose = RR_Hip;
    vicon_data_out_var.LF_Hip_pose = LR_Hip;

    vicon_data_out.publish(vicon_data_out_var);
}

void pose_info::quat_2_euler(const geometry_msgs::Quaternion msg, double *add_roll, double *add_pitch, double *add_yaw) {

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to access roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(*add_roll, *add_pitch, *add_yaw);
}

void pose_info::transformation_matrices(double roll, double pitch, double yaw, Eigen::Matrix4f *transformation_matrix) {

    Eigen::Matrix4f roll_tf;
    Eigen::Matrix4f pitch_tf;
    Eigen::Matrix4f yaw_tf;
    Eigen::Matrix4f translate_tf;

    roll_tf << 1, 0, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, 0, std::sin(roll), std::cos(roll), 0, 0, 0, 0, 1; //roll 3-D matrix
    pitch_tf << std::cos(pitch), 0, std::sin(pitch), 0, 0, 1, 0, 0, -std::sin(pitch), 0, std::cos(pitch), 0, 0, 0, 0 ,1; //pitch 3-D matrix
    yaw_tf << std::cos(yaw), -std::sin(yaw), 0, 0, std::sin(yaw), std::cos(yaw), 0 ,0, 0, 0, 1, 0, 0, 0, 0, 1; //yaw 3-D matrix

    *transformation_matrix = yaw_tf*pitch_tf*roll_tf;
}

void pose_info::getCoM(double x, double y, double z, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped *add_com_tf) {

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
    add_com_tf->pose.position.x = temp_pos(0,0);
    add_com_tf->pose.position.y = temp_pos(1,0);
    add_com_tf->pose.position.z = temp_pos(2,0);
}

void pose_info::getHips(const geometry_msgs::PoseStamped& com_tf, const Eigen::Matrix4f& transformation_matrix, geometry_msgs::PoseStamped *add_LF_Hip, geometry_msgs::PoseStamped *add_RF_Hip, geometry_msgs::PoseStamped *add_RR_Hip, geometry_msgs::PoseStamped *add_LR_Hip) {

    Eigen::MatrixXf temp_pos(4,1);
    Eigen::Matrix4f translation_to_origin;
    Eigen::Matrix4f translation_back;
    translation_to_origin << 1, 0, 0, -com_tf.pose.position.x, 0, 1, 0 ,-com_tf.pose.position.y, 0, 0, 1, -com_tf.pose.position.z, 0, 0, 0, 1;
    translation_back << 1, 0, 0, com_tf.pose.position.x, 0, 1, 0 ,com_tf.pose.position.y, 0, 0, 1, com_tf.pose.position.z, 0, 0, 0, 1;

    //LF hip pose
    temp_pos << com_tf.pose.position.x - robot_length / 2, com_tf.pose.position.y - robot_width / 2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_LF_Hip->pose.position.x = temp_pos(0,0);
    add_LF_Hip->pose.position.y = temp_pos(1,0);
    add_LF_Hip->pose.position.z = temp_pos(2,0);

    add_LF_Hip->pose.orientation = com_tf.pose.orientation;

    //RF hip pose
    temp_pos << com_tf.pose.position.x - robot_length / 2, com_tf.pose.position.y + robot_width / 2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_RF_Hip->pose.position.x = temp_pos(0,0);
    add_RF_Hip->pose.position.y = temp_pos(1,0);
    add_RF_Hip->pose.position.z = temp_pos(2,0);

    add_RF_Hip->pose.orientation = com_tf.pose.orientation;

    //RR hip pose
    temp_pos << com_tf.pose.position.x + robot_length / 2, com_tf.pose.position.y + robot_width / 2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_RR_Hip->pose.position.x = temp_pos(0,0);
    add_RR_Hip->pose.position.y = temp_pos(1,0);
    add_RR_Hip->pose.position.z = temp_pos(2,0);

    add_RR_Hip->pose.orientation = com_tf.pose.orientation;

    //LR hip pose
    temp_pos << com_tf.pose.position.x + robot_length / 2, com_tf.pose.position.y - robot_width / 2, com_tf.pose.position.z - hip_offset, 1;
    temp_pos = translation_back*transformation_matrix*translation_to_origin*temp_pos;

    add_LR_Hip->pose.position.x = temp_pos(0,0);
    add_LR_Hip->pose.position.y = temp_pos(1,0);
    add_LR_Hip->pose.position.z = temp_pos(2,0);

    add_LR_Hip->pose.orientation = com_tf.pose.orientation;
}

//Update log positions
void pose_info::find_log_positions(const geometry_msgs::PoseStamped::ConstPtr &msg, int i){

    this->log_positions[i] = msg->pose.position.x;
}

//Find closest obstacle to each hip
void pose_info::closestObstacle(double LF_x, double RF_x, double RR_x, double LR_x, int *LF_obs, int *RF_obs, int *RR_obs, int *LR_obs, double* LF_obs_dist, double* RF_obs_dist, double* RR_obs_dist, double* LR_obs_dist) {

    *LF_obs_dist = FLT_MAX;
    *RF_obs_dist = FLT_MAX;
    *RR_obs_dist = FLT_MAX;
    *LR_obs_dist = FLT_MAX;

    for(int i = 0; i < 15; i++){

        if(dist(LF_x,log_positions[i])< *LF_obs_dist){
            *LF_obs = i+1;
            *LF_obs_dist = dist(LF_x,log_positions[i]);
        }

        if(dist(RF_x,log_positions[i]) < *RF_obs_dist){
            *RF_obs = i+1;
            *RF_obs_dist = dist(RF_x,log_positions[i]);
        }

        if(dist(RR_x,log_positions[i]) < *RR_obs_dist){
            *RR_obs = i+1;
            *RR_obs_dist = dist(RR_x,log_positions[i]);
        }

        if(dist(LR_x,log_positions[i]) < *LR_obs_dist){
            *LR_obs = i+1;
            *LR_obs_dist = dist(LR_x,log_positions[i]);
        }
    }
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "pose_info");
    ros::NodeHandle n;
    pose_info pi;

    ros::spin();
    return 0;
}
