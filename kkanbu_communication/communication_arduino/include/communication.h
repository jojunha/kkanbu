#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <math.h>
#include <ros/ros.h>

#include <tf/tf.h>

#include "kkanbu_msgs/ControlCommand.h"
#include "kkanbu_msgs/VehicleState.h"
#include "kkanbu_msgs/current_state.h"

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#define PI 3.14159265358979323846 /* pi */

class Communication_Arduino{
    private:
        ros::NodeHandle nh_;

        ros::Publisher pub_VehicleState;

        ros::Publisher pub_VehicleOdom;

        ros::Subscriber sub_ControlCommand;
        ros::Subscriber sub_CurrentState;


    public:
        Communication_Arduino();
        ~Communication_Arduino();
        
        
        // Callback Func
        void get_ControlCommand(const kkanbu_msgs::ControlCommand::ConstPtr &msg);
        void get_CurrentState(const kkanbu_msgs::current_state::ConstPtr &msg);
        
        // Func
        // For High Level Control Test 
        void Update_VehicleState();

        void Publish_VehicleState();

    private:
        // Input 
        kkanbu_msgs::ControlCommand control_cmd_;
        kkanbu_msgs::current_state current_state_;
        
        // Output
        kkanbu_msgs::VehicleState ego_state_;

        // Variable
        nav_msgs::Odometry ego_odom_;
        double ego_init_x_, ego_init_y_, ego_init_yaw_; 
        bool isControlCmd = false;
        double prevRosTime_;

        // double ego_wheelBase_ = 0.6;
        double ego_front_ = 0.3;
        double ego_rear_ = 0.3;
        double ego_yaw_ = 0.0;
};

#endif