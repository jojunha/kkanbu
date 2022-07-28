#include "communication.h"

Communication_Arduino::Communication_Arduino(){
    // Get Parameter from launch
    nh_.param("communication_arduino/ego/init_x", ego_init_x_, 0.0);
    nh_.param("communication_arduino/ego/init_y", ego_init_y_, 0.0);
    nh_.param("communication_arduino/ego/init_yaw", ego_init_yaw_, 0.0);
    
    // Publisher
    pub_VehicleState = nh_.advertise<kkanbu_msgs::VehicleState>("/ego/vehicle_state", 10);
    pub_VehicleOdom = nh_.advertise<nav_msgs::Odometry>("/ego/odometry", 10);

    // Subscriber
    sub_ControlCommand = nh_.subscribe("/ego/control_cmd",10,&Communication_Arduino::get_ControlCommand, this);
    sub_CurrentState = nh_.subscribe("/current_state",10,&Communication_Arduino::get_CurrentState, this);
    

    // Init State
    ego_odom_.header.stamp = ros::Time::now();
    ego_odom_.header.frame_id = "/map";
    ego_odom_.child_frame_id = "/ego";
    
    ego_odom_.pose.pose.position.x = ego_init_x_;
    ego_odom_.pose.pose.position.y = ego_init_y_;
    ego_odom_.pose.pose.position.z = 0.0;
    // 처음 yaw 값이 이상함!
    // ego_yaw_ = ego_init_yaw_ + PI;
    ego_yaw_ = atan2(-1.49833577982+1.48723699614 ,2.17403534976-2.1828728913) + PI*0.675;
    tf::Quaternion q;
    q.setRPY(0, 0, ego_init_yaw_);
    // q.normalize();

    ego_odom_.pose.pose.orientation.x = q.getX();
    ego_odom_.pose.pose.orientation.y = q.getY();
    ego_odom_.pose.pose.orientation.z = q.getZ();
    ego_odom_.pose.pose.orientation.w = q.getW();

    ego_odom_.twist.twist.linear.x = 0.0;
}

Communication_Arduino::~Communication_Arduino(){}

void Communication_Arduino::get_ControlCommand(const kkanbu_msgs::ControlCommand::ConstPtr &msg){
    control_cmd_ = *msg;
    isControlCmd = true;
}

void Communication_Arduino::get_CurrentState(const kkanbu_msgs::current_state::ConstPtr &msg){
    current_state_ = *msg;
    // isCurrentState = true;
}

void Communication_Arduino::Update_VehicleState(){
    // Calculate dt_ by dividing the previous time and the current time
    double dt_ = ros::Time::now().toSec() - prevRosTime_;
    // if (dt_ < 0) {
    // return false;
    // }

    prevRosTime_ = ros::Time::now().toSec();
    
    ego_odom_.twist.twist.linear.x = ego_odom_.twist.twist.linear.x + dt_*(control_cmd_.accel - control_cmd_.brake);
    // ego_odom_.twist.twist.linear.x = current_state_.velocity;

    double beta = atan2((ego_front_ * tan(control_cmd_.steering)),(ego_front_ + ego_rear_));

    double dx = dt_ * ego_odom_.twist.twist.linear.x * cos(ego_yaw_ + beta);
    double dy = dt_ * ego_odom_.twist.twist.linear.x * sin(ego_yaw_ + beta);
    double dyaw = dt_ * ego_odom_.twist.twist.linear.x * cos(beta)/(ego_front_ + ego_rear_)*(tan(control_cmd_.steering));

    ego_yaw_ += dyaw;
    ego_odom_.pose.pose.position.x += dx;
    ego_odom_.pose.pose.position.y += dy;
    
    // ego_odom_.pose.pose.position.x = current_state_.x;
    // ego_odom_.pose.pose.position.y = current_state_.y;
    

    tf::Quaternion q;
    q.setRPY(0, 0, ego_yaw_);
    // q.normalize();

    ego_odom_.pose.pose.orientation.x = q.getX();
    ego_odom_.pose.pose.orientation.y = q.getY();
    ego_odom_.pose.pose.orientation.z = q.getZ();
    ego_odom_.pose.pose.orientation.w = q.getW();

    ego_state_.x = ego_odom_.pose.pose.position.x;
    ego_state_.y = ego_odom_.pose.pose.position.y;
    ego_state_.yaw = ego_yaw_;
    ego_state_.velocity = ego_odom_.twist.twist.linear.x;

}

void Communication_Arduino::Publish_VehicleState(){
    Update_VehicleState();
    pub_VehicleOdom.publish(ego_odom_);
    pub_VehicleState.publish(ego_state_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "communication_arduino");
  Communication_Arduino ca;

  // The approximate control time is 100 Hz
  ros::Rate loop_rate(100);
  
  while (ros::ok()) {
    ca.Publish_VehicleState();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

