#include "control_kkanbu.h"

Control_Robot::Control_Robot(){
    // Param
    nh_.param("kkanbu_control/ego/target_velocity", target_velocity_, 0.0);
    nh_.param("kkanbu_control/ego/p_gain", p_gain_, 0.0);
    nh_.param("kkanbu_control/ego/lookAhead", lookAhead_, 0);
    
    // Publisher
    pub_ControlCommand = nh_.advertise<kkanbu_msgs::ControlCommand>("/ego/control_cmd",10);
    pub_ArduinoCommand = nh_.advertise<kkanbu_msgs::ControlCommand>("/ego/arduino_cmd",10);
    pub_lookAheadPose = nh_.advertise<visualization_msgs::Marker>("/ego/lookahead_pose",10);
    // Subscriber
    sub_VehicleState = nh_.subscribe("/current_state",10,&Control_Robot::get_VehicleState,this);
    // sub_VehicleState = nh_.subscribe("/ego/vehicle_state",10,&Control_Robot::get_VehicleState,this);
    sub_LocalPath = nh_.subscribe("/local/best_path",10,&Control_Robot::get_LocalPath,this);
}

Control_Robot::~Control_Robot(){}

// void Control_Robot::get_VehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg){
//     vehicle_state_ = *msg;
//     // vehicle_state_.yaw = vehicle_state_.yaw*PI/180;
// }

void Control_Robot::get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg){
    vehicle_state_ = *msg;
}

void Control_Robot::get_LocalPath(const nav_msgs::Path::ConstPtr& msg){
    best_local_path_ = *msg;
    isPath = true;
}

void Control_Robot::LongitudinalControl(){
    double error_velocity = target_velocity_ - vehicle_state_.velocity;
    if(error_velocity < 0){
        control_cmd_.accel = 0;
        arduino_control_cmd_.accel = 70;
        control_cmd_.brake = fabs(error_velocity) * p_gain_;
    }else{
        control_cmd_.accel = error_velocity;//error_velocity * p_gain_;
        arduino_control_cmd_.accel = 115 + error_velocity;
        control_cmd_.brake = 0;
    }
}

geometry_msgs::PoseStamped Control_Robot::transformEgo2World(geometry_msgs::PoseStamped ego_frame_pose){
    geometry_msgs::PoseStamped world_frame_pose;
    world_frame_pose.header.frame_id = "/map";
    try{
        ego_frame_pose.header.frame_id = "/ego";
        ego2map.transformPose("/map",ego_frame_pose,world_frame_pose);
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        // continue;
    }
    return world_frame_pose;
}

geometry_msgs::PoseStamped Control_Robot::transformWorld2Ego(geometry_msgs::PoseStamped world_frame_pose){
    geometry_msgs::PoseStamped ego_frame_pose;
    ego_frame_pose.header.frame_id = "/ego";
    try{
        world_frame_pose.header.frame_id = "/map";
        ego2map.transformPose("/ego",world_frame_pose,ego_frame_pose);
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        // continue;
    }
    return ego_frame_pose;
}

void Control_Robot::Make_LookAheadMarker(geometry_msgs::PoseStamped lookAhead_Pose){
    lookAhead_pose_maker_.header.frame_id = "/map";
    lookAhead_pose_maker_.header.stamp = ros::Time::now();
    // lookAhead_pose_maker_.ns = "Lateral Control";
    // lookAhead_pose_maker_.id = 0;
    lookAhead_pose_maker_.type = visualization_msgs::Marker::SPHERE;
    lookAhead_pose_maker_.pose.position.x = lookAhead_Pose.pose.position.x;
    lookAhead_pose_maker_.pose.position.y = lookAhead_Pose.pose.position.y;
    lookAhead_pose_maker_.scale.x = 0.5;
    lookAhead_pose_maker_.scale.y = 0.5;
    lookAhead_pose_maker_.scale.z = 0.5;
    lookAhead_pose_maker_.color.r = 1.0;
    lookAhead_pose_maker_.color.g = 0.0;
    lookAhead_pose_maker_.color.b = 0.0;
    lookAhead_pose_maker_.color.a = 1.0;
}

void Control_Robot::LateralControl(){
    if(isPath){
        lookAhead_idx_ = closest_idx_ + lookAhead_;
        lookAhead_Pose_ = transformEgo2World(best_local_path_.poses[lookAhead_idx_]);
        Make_LookAheadMarker(lookAhead_Pose_);
        geometry_msgs::PoseStamped current_state;
        current_state.header.frame_id = "/map";
        current_state.pose.position.x = vehicle_state_.x;
        current_state.pose.position.y = vehicle_state_.y;
        current_state.pose.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0,0,vehicle_state_.yaw);
        q.normalize();
        current_state.pose.orientation.x = q[0];
        current_state.pose.orientation.y = q[1];
        current_state.pose.orientation.z = q[2];
        current_state.pose.orientation.w = q[3];
        geometry_msgs::PoseStamped current_state_ego = transformWorld2Ego(current_state);
        double lookAhead_distance = sqrt(pow(vehicle_state_.x - lookAhead_Pose_.pose.position.x,2)+pow(vehicle_state_.y - lookAhead_Pose_.pose.position.y,2));
        // double alpha = atan2(lookAhead_Pose_.pose.position.y -vehicle_state_.y,lookAhead_Pose_.pose.position.x -vehicle_state_.x ) - vehicle_state_.yaw;
        double cross_track_error = best_local_path_.poses[lookAhead_idx_].pose.position.y;
        // control_cmd_.steering = atan2(2 * wheelBase_ * sin(alpha),lookAhead_distance);
        control_cmd_.steering = atan2(2 * wheelBase_ * cross_track_error,pow(lookAhead_distance,2));
        printf("Cross Track Error : %f, Steering : %f \n", cross_track_error,control_cmd_.steering*180/PI);
        arduino_control_cmd_.steering = -(control_cmd_.steering * (180/PI) * 10) + 101;//-();
    }else{
        control_cmd_.steering = 0;
        ROS_WARN("Path isn't subscribed yet!");
    }
    
}

void Control_Robot::Publish_ControlCommand(){
    LongitudinalControl();
    LateralControl();
    pub_ControlCommand.publish(control_cmd_);
    pub_ArduinoCommand.publish(arduino_control_cmd_);
    pub_lookAheadPose.publish(lookAhead_pose_maker_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kkanbu_control");
  Control_Robot cr;

  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
  
  while (ros::ok()) {
    cr.Publish_ControlCommand();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}



