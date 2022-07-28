#ifndef CONTROL_KKANBU_H
#define CONTROL_KKANBU_H
#include <math.h>
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "kkanbu_msgs/ControlCommand.h"
#include "kkanbu_msgs/VehicleState.h"
#include "kkanbu_msgs/current_state.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#define PI 3.14159265358979323846 /* pi */

class Control_Robot{
    private:
        ros::NodeHandle nh_;
        
        ros::Publisher pub_ControlCommand;
        ros::Publisher pub_ArduinoCommand;
        ros::Publisher pub_lookAheadPose;

        ros::Subscriber sub_VehicleState;
        ros::Subscriber sub_LocalPath;

    public:
        Control_Robot();
        ~Control_Robot();

        // Callback Func
        // void get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg);
        // void get_VehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg);
        void get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg);
        void get_LocalPath(const nav_msgs::Path::ConstPtr& msg);
        
        //Func
        void LongitudinalControl();
        void LateralControl();
        void Make_LookAheadMarker(geometry_msgs::PoseStamped lookAhead_Pose);
        void Publish_ControlCommand();
        geometry_msgs::PoseStamped transformEgo2World(geometry_msgs::PoseStamped);
        geometry_msgs::PoseStamped transformWorld2Ego(geometry_msgs::PoseStamped world_frame_pose);

    private:
        // Param
        double target_velocity_;
        double p_gain_;
        int lookAhead_;

        // Input
        kkanbu_msgs::current_state vehicle_state_;
        // kkanbu_msgs::VehicleState vehicle_state_;
        nav_msgs::Path best_local_path_;

        // TF
        tf::TransformListener ego2map;
        tf::StampedTransform transform_ego2map;

        //Output
        kkanbu_msgs::ControlCommand control_cmd_;
        kkanbu_msgs::ControlCommand arduino_control_cmd_;
        visualization_msgs::Marker lookAhead_pose_maker_;

        // Variables
        int closest_idx_ = 0;
        int lookAhead_idx_;
        double lookAhead_yaw_;
        double wheelBase_ = 0.6;
        geometry_msgs::PoseStamped lookAhead_Pose_;
        bool isPath = false;
};

#endif