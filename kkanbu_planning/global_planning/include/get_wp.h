#ifndef GET_WP_H
#define GET_WP_H

//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>
#include<queue>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>

//Message header
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int32.h>

#include "kkanbu_msgs/LandmarkPoint.h"
#include "kkanbu_msgs/LandmarkPath.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define INF 1e9

using namespace std;

class Entire_Path{
    private:
        ros::NodeHandle nh_;
        
        ros::Publisher pub_entirepath;
        ros::Subscriber sub_LandmarkPoint;
        ros::Subscriber sub_dstPoint;

    public:
        Entire_Path();
        ~Entire_Path();

        // Callback Func
        void Get_LandmarkPoint(const kkanbu_msgs::LandmarkPath::ConstPtr& msg);
        void Get_dstPoint(const kkanbu_msgs::LandmarkPoint::ConstPtr& msg);
        //Func
        void Save_Node();
        void Get_Graph();

        void Get_ShortestPath(int start, int dst);
        void Publish_entirepath();
        bool land_ = false;
        int d[10001];
        //vector<vector<float>> shortest_path;

    public:
        //Variables
        struct Node_Info{
            float x,y,yaw;
            int point_num;
        };
        vector<Node_Info> node;
        vector<pair<int, double> > graph[1001];
        
       // Node_Info start;
       // Node_Info dst;

        //Input
        kkanbu_msgs::LandmarkPath landmark_point_;
        kkanbu_msgs::LandmarkPoint dst_point_;

        ///Output
        // kkanbu_msgs::LandmarkPath landmark_path_;
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;
        tf::Quaternion q;
        //geometry_msgs::Quaternion quat_msg;
        bool isSaveNodeDone = false;
        bool isGetGraphDone = false;
        bool isShortestPathDone = false;
};



#endif