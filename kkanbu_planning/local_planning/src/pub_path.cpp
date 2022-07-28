//STD header
#include<vector>
#include<cmath>
#include<string>
#include<iostream>

//ROS header
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>

//Message header
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Int32.h>
#include "kkanbu_msgs/SensorPointArray.h"
#include "kkanbu_msgs/SensorPoint.h"
#include "kkanbu_msgs/VehicleState.h"
#include "kkanbu_msgs/current_state.h"
#include<visualization_msgs/Marker.h>



//Using
using namespace std;

bool execute_cal = true;

class pointPub{
    public:
        pointPub(){
            
                // 초기 위치 받아옴
            // sub_x_goal= nh.subscribe("/x_slot",10,&pointPub::x_Callback,this); 
            // sub_y_goal= nh.subscribe("/y_slot",10,&pointPub::y_Callback,this);
            sub_obstacle=nh.subscribe("/final_circle",1000,&pointPub::get_Hurdle,this);
	        //     Publisher 객체 선언, topic의 이름 xy_pose, 메시지 큐의 크기가 100
            pub_pose=nh.advertise<nav_msgs::Path>("/local/best_path",10);
            pub_lookahead_=nh.advertise<visualization_msgs::Marker>("/local/lookahead",10);
            pub_candidate_1 = nh.advertise<nav_msgs::Path>("/local/candidate_path_1",10);
            pub_candidate_2 = nh.advertise<nav_msgs::Path>("/local/candidate_path_2",10);
            pub_candidate_3 = nh.advertise<nav_msgs::Path>("/local/candidate_path_3",10);
            pub_candidate_4 = nh.advertise<nav_msgs::Path>("/local/candidate_path_4",10);

            sub_VehicleState = nh.subscribe("/current_state",10,&pointPub::get_VehicleState,this);
            sub_GlobalPath = nh.subscribe("/map/best_entire_path",10,&pointPub::get_GlobalPath,this);


            path_1.header.frame_id = "/ego";
            path_2.header.frame_id = "/ego";
            path_3.header.frame_id = "/ego";
            path_4.header.frame_id = "/ego";
            straight_path.header.frame_id = "/ego";
            
             // rviz에서 frame을 map으로 정해줌
        }
        ~pointPub(){}
    private:
        ros::NodeHandle nh;  // 시스템과의 소통을 위해 객체 선언

        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped init;
        nav_msgs::Path path_1;
        nav_msgs::Path path_2;
        nav_msgs::Path path_3;
        nav_msgs::Path path_4;
        nav_msgs::Path straight_path;
        kkanbu_msgs::SensorPointArray HurdleArray;
        nav_msgs::Path global_path;
        // kkanbu_msgs::VehicleState vehicle_state_;
        kkanbu_msgs::current_state vehicle_state_;
        visualization_msgs::Marker lookahead_;

        // ros::Subscriber sub_x_goal;
        // ros::Subscriber sub_y_goal;
        ros::Subscriber sub_obstacle;
        ros::Subscriber sub_VehicleState;
        ros::Subscriber sub_LocalPath;
        ros::Subscriber sub_GlobalPath;

        ros::Publisher pub_pose;
        ros::Publisher pub_lookahead_;
        ros::Publisher pub_candidate_1;
        ros::Publisher pub_candidate_2;
        ros::Publisher pub_candidate_3;
        ros::Publisher pub_candidate_4;
  

        tf::Quaternion slope_1;
        tf::Quaternion slope_2;
        tf::Quaternion slope_3;
        tf::Quaternion slope_4;  
        tf::Quaternion straight_slope;
        tf::TransformListener map2ego;
  

        double vehicle_r = 0.5; //
        // double hurdle_x;
        // double hurdle_y;
        // double hurdle_r;
        // bool hurdle_state;    
        int obs_num;
        bool path_collision[5]={true,true,true,true,true};

        // 경로 계산에 쓰이는 변수들
        double r1=17;
        double r2=25.5;
        double r3=10;
        double r4=15;
        double angle_1;
        double angle_2;
        double PI = 3.141592;
        double m;
        double c,d;
        double second_m;
        double third_m;
        double fourth_m;
        double straight_m;
        double pass_through_x;
        double pass_through_y;
        double pass_through_x_2;
        double pass_through_y_2;
        int point_num = 75;     // 초기 점과 최종 점을 포함하여 2*k개의 점을 찍어서 path를 생성할 것
        double narrow_point_info[11][3];
        double wide_point_info[11][3];
        double straight_point_info[11][2];

        // int closest_idx_;
        int sum_possible = 0;
        bool GLPATH = false;
        int lookAhead_idx_;
        int centerline_cost_idx_;
        geometry_msgs::PoseStamped lookAhead_Pose_;
        geometry_msgs::PoseStamped centerline_cost_pose_;
        double local_distance[5]={1000,1000,1000,1000,1000};
        double dx[5]={-10.-5,5,10,0};
        double dy=20;
        int proper_idx;

        double global_slope;
        double slope_line;

        
    public:
        void get_Hurdle(const kkanbu_msgs::SensorPointArray::ConstPtr& msg){
            HurdleArray = *msg;
        }
        // Callback 함수들
        // void get_VehicleState(const kkanbu_msgs::VehicleState::ConstPtr& msg){
        //     vehicle_state_ = *msg;
        // }
        void get_VehicleState(const kkanbu_msgs::current_state::ConstPtr& msg){
            vehicle_state_ = *msg;
        }


        void get_GlobalPath(const nav_msgs::Path::ConstPtr& msg){
            global_path = *msg;
            // ROS_INFO("I Got Path");
            if(global_path.poses.size()>0){
                GLPATH = true;
            }
            // else{
            //     GLPATH = false;
            // }
        }

        void find_ClosestWaypoint(){
            
            // if(global_path.poses.size() > 0){
            //     GLPATH = true;
            // }else{
            //     GLPATH = false;
            // }
            // bool a = false;
            if(GLPATH){
                printf("global path size : %d\n",global_path.poses.size());
                // if(global_path.poses.size() > 0){
                int closest_idx_ = 0;
                double min_distance = 100000000;
                for(int i=0;i<global_path.poses.size();i++){
                    // printf("wow1\n");
                    double temp_distance = sqrt(pow(vehicle_state_.x - global_path.poses[i].pose.position.x,2)+pow(vehicle_state_.y - global_path.poses[i].pose.position.y,2));
                    // printf("wow2\n");
                    // printf("temp distance : %f, min Distance : %f \n", temp_distance,min_distance);
                    if(temp_distance < min_distance){
                        closest_idx_ = i;
                        min_distance = temp_distance;
                        // printf("Closest Idx is changed to %d \n", closest_idx_);
                    }
                }
                
                lookAhead_idx_ = closest_idx_ + 8;  // 인덱스 사이의 거리가 1미터 안쪽임
                centerline_cost_idx_ = lookAhead_idx_;
                printf("Closest Idx : %d , LookAhead Idx : %d\n", closest_idx_, lookAhead_idx_);
                // cout << global_path << "\n";
                
                geometry_msgs::PoseStamped ego_frame_pose;
                ego_frame_pose.header.frame_id = "/ego";
                geometry_msgs::PoseStamped world_frame_pose;
                world_frame_pose.header.frame_id = "/map";
                // printf("wow1\n");
                world_frame_pose.pose.position.x =global_path.poses[lookAhead_idx_].pose.position.x;
                world_frame_pose.pose.position.y =global_path.poses[lookAhead_idx_].pose.position.y;
                world_frame_pose.pose.position.z =global_path.poses[lookAhead_idx_].pose.position.z;
                world_frame_pose.pose.orientation = global_path.poses[lookAhead_idx_].pose.orientation;
                printf("World Frame Pose... x : %f , y : %f \n",world_frame_pose.pose.position.x,world_frame_pose.pose.position.y);
                    
                try{
                    map2ego.transformPose("/ego",world_frame_pose,ego_frame_pose);
                }catch (tf::TransformException &ex){
                    ROS_ERROR("%s",ex.what());
                }
                lookAhead_Pose_.pose.position.x = ego_frame_pose.pose.position.x;
                lookAhead_Pose_.pose.position.y = ego_frame_pose.pose.position.y;
                // lookAhead_Pose_ = global_path.poses[lookAhead_idx_];
                printf("Ego Frame Pose... x : %f , y : %f \n",lookAhead_Pose_.pose.position.x,lookAhead_Pose_.pose.position.y);
                
                geometry_msgs::PoseStamped ego_frame_pose_;
                ego_frame_pose_.header.frame_id = "/ego";
                geometry_msgs::PoseStamped world_frame_pose_;
                world_frame_pose_.header.frame_id = "/map";
                // printf("wow1\n");
                world_frame_pose_.pose.position.x =global_path.poses[centerline_cost_idx_].pose.position.x;
                world_frame_pose_.pose.position.y =global_path.poses[centerline_cost_idx_].pose.position.y;
                world_frame_pose_.pose.position.z =global_path.poses[centerline_cost_idx_].pose.position.z;
                world_frame_pose_.pose.orientation = global_path.poses[centerline_cost_idx_].pose.orientation;
                // printf("World Frame Pose... x : %f , y : %f \n",world_frame_pose.pose.position.x,world_frame_pose.pose.position.y);
                    
                try{
                    map2ego.transformPose("/ego",world_frame_pose_,ego_frame_pose_);
                }catch (tf::TransformException &ex){
                    ROS_ERROR("%s",ex.what());
                }
                centerline_cost_pose_.pose.position.x = ego_frame_pose_.pose.position.x;
                centerline_cost_pose_.pose.position.y = ego_frame_pose_.pose.position.y;
                
                cal_slope();
                // }
            }
            // printf("wow Wow \n");
            // measure_distance();
        }

        void cal_slope(){
            double den = lookAhead_Pose_.pose.position.x;
            if(den < 0.001){
                den = 0.001;
            }
            // global_slope = (lookAhead_Pose_.pose.position.y-vehicle_state_.y)/(den);
            slope_line = lookAhead_Pose_.pose.position.y/den;
            cal_point();
        }

        void cal_point(){
            nav_msgs::Path local_path_1;
            nav_msgs::Path local_path_2;
            nav_msgs::Path local_path_3;
            nav_msgs::Path local_path_4;
            nav_msgs::Path local_path_st;

            local_path_1.header.frame_id = "/ego";
            local_path_2.header.frame_id = "/ego";
            local_path_3.header.frame_id = "/ego";
            local_path_4.header.frame_id = "/ego";
            local_path_st.header.frame_id = "/ego";

            path_1 = local_path_1;
            path_2 = local_path_2;
            path_3 = local_path_3;
            path_4 = local_path_4;
            double lookAhead_distance = sqrt(pow(lookAhead_Pose_.pose.position.x,2)+pow(lookAhead_Pose_.pose.position.x,2));
            double narrow_offset = 0.5;
            double wide_offset = 1.0;
            double lookahead_theta = atan2(lookAhead_Pose_.pose.position.y,lookAhead_Pose_.pose.position.x);
            double narrow_radius = lookAhead_distance;
            double wide_radius = lookAhead_distance;
            double narrow_alpha = acos(1-(narrow_offset/(2*narrow_radius)));
            double wide_alpha = acos(1-(wide_offset/(2*wide_radius)));
            
            printf("Narrow R : %f, Wide R : %f \n", narrow_radius,wide_radius);
            tf::Quaternion q;
            q.setRPY(0.0,0.0,lookahead_theta);
            q.normalize();
            tf::Matrix3x3 rotation_ego2lookAhead(q);
            // rotation_ego2lookAhead.setRPY(0.0,0.0,lookahead_theta);
            printf("global theta : %f\n", lookahead_theta);
            
        //     // printf("wow1\n");
            for(int j=0;j<point_num;j++){   // 초기 점과 최종 점을 포함하여 k개의 점을 찍어서 path를 생성할 것
                // Narrow (+) : 1 , (-) : 2
                geometry_msgs::PoseStamped local_point_1;
                geometry_msgs::PoseStamped local_point_2;
                // Wide (+) : 3 , (-) : 4
                geometry_msgs::PoseStamped local_point_3;
                geometry_msgs::PoseStamped local_point_4;
                geometry_msgs::PoseStamped local_point_straight;

                local_point_1.header.frame_id = "/ego";
                local_point_2.header.frame_id = "/ego";
                local_point_3.header.frame_id = "/ego";
                local_point_4.header.frame_id = "/ego";
                local_point_straight.header.frame_id = "/ego";
                // --------------------------------- 직선 뽑기 ----------------------------------------------//
                local_point_straight.pose.position.x = (lookAhead_Pose_.pose.position.x/point_num) * j;
                local_point_straight.pose.position.y = (lookAhead_Pose_.pose.position.y/point_num) * j;
                straight_slope = tf::createQuaternionFromRPY(0,0,lookahead_theta);
                local_point_straight.pose.orientation.x = straight_slope[0];
                local_point_straight.pose.orientation.y = straight_slope[1];
                local_point_straight.pose.orientation.z = straight_slope[2];
                local_point_straight.pose.orientation.w = straight_slope[3];
                local_path_st.poses.push_back(local_point_straight);
                // -----------------------------------------------------------------------------------------//
                
                if(j<25){
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_1.pose.position.y = narrow_radius*(1 - cos(j*narrow_alpha/(point_num/3)));
                    local_point_1.pose.position.x = narrow_radius*sin(j*narrow_alpha/(point_num/3));
                    slope_1 = tf::createQuaternionFromRPY(0,0,j*narrow_alpha/(point_num/3) + lookahead_theta);
                    local_point_1.pose.orientation.x = slope_1[0];
                    local_point_1.pose.orientation.y = slope_1[1];
                    local_point_1.pose.orientation.z = slope_1[2];
                    local_point_1.pose.orientation.w = slope_1[3];
                    tf::Vector3 point_1(local_point_1.pose.position.x,local_point_1.pose.position.y,0.0);
                    point_1 = rotation_ego2lookAhead*point_1;
                    local_point_1.pose.position.x = point_1.getX();
                    local_point_1.pose.position.y = point_1.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 오른쪽 얕은 경로 뽑기 (-) -----------------------------------//
                    local_point_2.pose.position.y = -narrow_radius*(1 - cos(j*narrow_alpha/(point_num/3)));
                    local_point_2.pose.position.x = narrow_radius*sin(j*narrow_alpha/(point_num/3));
                    slope_2 = tf::createQuaternionFromRPY(0,0,2*M_PI-j*narrow_alpha/(point_num/3)+lookahead_theta);
                    local_point_2.pose.orientation.x = slope_2[0];
                    local_point_2.pose.orientation.y = slope_2[1];
                    local_point_2.pose.orientation.z = slope_2[2];
                    local_point_2.pose.orientation.w = slope_2[3];
                    tf::Vector3 point_2(local_point_2.pose.position.x,local_point_2.pose.position.y,0.0);
                    point_2 = rotation_ego2lookAhead*point_2;
                    local_point_2.pose.position.x = point_2.getX();
                    local_point_2.pose.position.y = point_2.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 왼쪽 넓은 경로 뽑기 (+) -----------------------------------//
                    local_point_3.pose.position.y = wide_radius*(1 - cos(j*wide_alpha/(point_num/3)));
                    local_point_3.pose.position.x = wide_radius*sin(j*wide_alpha/(point_num/3));
                    slope_3 = tf::createQuaternionFromRPY(0,0,j*wide_alpha/(point_num/3)+lookahead_theta);
                    local_point_3.pose.orientation.x = slope_3[0];
                    local_point_3.pose.orientation.y = slope_3[1];
                    local_point_3.pose.orientation.z = slope_3[2];
                    local_point_3.pose.orientation.w = slope_3[3];
                    tf::Vector3 point_3(local_point_3.pose.position.x,local_point_3.pose.position.y,0.0);
                    point_3 = rotation_ego2lookAhead*point_3;
                    local_point_3.pose.position.x = point_3.getX();
                    local_point_3.pose.position.y = point_3.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 오른쪽 넓은 경로 뽑기 (-) -----------------------------------//
                    local_point_4.pose.position.y = -wide_radius*(1 - cos(j*wide_alpha/(point_num/3)));
                    local_point_4.pose.position.x = wide_radius*sin(j*wide_alpha/(point_num/3));
                    slope_4 = tf::createQuaternionFromRPY(0,0,2*M_PI-j*wide_alpha/(point_num/3)+lookahead_theta);
                    local_point_4.pose.orientation.x = slope_4[0];
                    local_point_4.pose.orientation.y = slope_4[1];
                    local_point_4.pose.orientation.z = slope_4[2];
                    local_point_4.pose.orientation.w = slope_4[3];
                    tf::Vector3 point_4(local_point_4.pose.position.x,local_point_4.pose.position.y,0.0);
                    point_4 = rotation_ego2lookAhead*point_4;
                    local_point_4.pose.position.x = point_4.getX();
                    local_point_4.pose.position.y = point_4.getY();
                // -----------------------------------------------------------------------------------------//
                }else if(j>=25 && j < 50){
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_1.pose.position.y = narrow_radius*(1-cos(narrow_alpha)) + narrow_radius*(cos(narrow_alpha-(narrow_alpha*(j-25)/(point_num/2))) - cos(narrow_alpha));
                    local_point_1.pose.position.x = (narrow_radius*sin(narrow_alpha)) + narrow_radius*(sin(narrow_alpha)-sin(narrow_alpha-(narrow_alpha*(j-25)/(point_num/2))));
                    slope_1 = tf::createQuaternionFromRPY(0,0,(narrow_alpha-(narrow_alpha*(j-25)/(point_num/3)))+lookahead_theta);
                    local_point_1.pose.orientation.x = slope_1[0];
                    local_point_1.pose.orientation.y = slope_1[1];
                    local_point_1.pose.orientation.z = slope_1[2];
                    local_point_1.pose.orientation.w = slope_1[3];
                    tf::Vector3 point_1(local_point_1.pose.position.x,local_point_1.pose.position.y,0.0);
                    point_1 = rotation_ego2lookAhead*point_1;
                    local_point_1.pose.position.x = point_1.getX();
                    local_point_1.pose.position.y = point_1.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 오른쪽 얕은 경로 뽑기 (-) -----------------------------------//
                    local_point_2.pose.position.y = -(narrow_radius*(1-cos(narrow_alpha)) + narrow_radius*(cos(narrow_alpha-(narrow_alpha*(j-25)/(point_num/2))) - cos(narrow_alpha)));
                    local_point_2.pose.position.x = -(-(narrow_radius*sin(narrow_alpha)) - narrow_radius*(sin(narrow_alpha)-sin(narrow_alpha-(narrow_alpha*(j-25)/(point_num/2)))));
                    slope_2 = tf::createQuaternionFromRPY(0,0,2*M_PI-(narrow_alpha-(narrow_alpha*(j-25)/(point_num/3)))+lookahead_theta);
                    local_point_2.pose.orientation.x = slope_2[0];
                    local_point_2.pose.orientation.y = slope_2[1];
                    local_point_2.pose.orientation.z = slope_2[2];
                    local_point_2.pose.orientation.w = slope_2[3];
                    tf::Vector3 point_2(local_point_2.pose.position.x,local_point_2.pose.position.y,0.0);
                    point_2 = rotation_ego2lookAhead*point_2;
                    local_point_2.pose.position.x = point_2.getX();
                    local_point_2.pose.position.y = point_2.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_3.pose.position.y = wide_radius*(1-cos(wide_alpha)) + wide_radius*(cos(wide_alpha-(wide_alpha*(j-25)/(point_num/2))) - cos(wide_alpha));
                    local_point_3.pose.position.x = (wide_radius*sin(wide_alpha)) + wide_radius*(sin(wide_alpha)-sin(wide_alpha-(wide_alpha*(j-25)/(point_num/2))));
                    slope_3 = tf::createQuaternionFromRPY(0,0,(wide_alpha-(wide_alpha*(j-25)/(point_num/3)))+lookahead_theta);
                    local_point_3.pose.orientation.x = slope_3[0];
                    local_point_3.pose.orientation.y = slope_3[1];
                    local_point_3.pose.orientation.z = slope_3[2];
                    local_point_3.pose.orientation.w = slope_3[3];
                    tf::Vector3 point_3(local_point_3.pose.position.x,local_point_3.pose.position.y,0.0);
                    point_3 = rotation_ego2lookAhead*point_3;
                    local_point_3.pose.position.x = point_3.getX();
                    local_point_3.pose.position.y = point_3.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 오른쪽 얕은 경로 뽑기 (-) -----------------------------------//
                    local_point_4.pose.position.y = -(wide_radius*(1-cos(wide_alpha)) + wide_radius*(cos(wide_alpha-(wide_alpha*(j-25)/(point_num/2))) - cos(wide_alpha)));
                    local_point_4.pose.position.x = -(-(wide_radius*sin(wide_alpha)) - wide_radius*(sin(wide_alpha)-sin(wide_alpha-(wide_alpha*(j-25)/(point_num/2)))));
                    slope_4 = tf::createQuaternionFromRPY(0,0,2*M_PI-(wide_alpha-(wide_alpha*(j-25)/(point_num/3)))+lookahead_theta);
                    local_point_4.pose.orientation.x = slope_4[0];
                    local_point_4.pose.orientation.y = slope_4[1];
                    local_point_4.pose.orientation.z = slope_4[2];
                    local_point_4.pose.orientation.w = slope_4[3];
                    tf::Vector3 point_4(local_point_4.pose.position.x,local_point_4.pose.position.y,0.0);
                    point_4 = rotation_ego2lookAhead*point_4;
                    local_point_4.pose.position.x = point_4.getX();
                    local_point_4.pose.position.y = point_4.getY();
                // -----------------------------------------------------------------------------------------//
                }else{
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_1.pose.position.y = narrow_offset;
                    local_point_1.pose.position.x = 2*narrow_radius*sin(narrow_alpha) + (lookAhead_distance - 2*narrow_radius*sin(narrow_alpha))*(j-50)/(point_num/3);
                    slope_1 = tf::createQuaternionFromRPY(0,0,M_PI/2);
                    local_point_1.pose.orientation.x = slope_1[0];
                    local_point_1.pose.orientation.y = slope_1[1];
                    local_point_1.pose.orientation.z = slope_1[2];
                    local_point_1.pose.orientation.w = slope_1[3];
                    tf::Vector3 point_1(local_point_1.pose.position.x,local_point_1.pose.position.y,0.0);
                    point_1 = rotation_ego2lookAhead*point_1;
                    local_point_1.pose.position.x = point_1.getX();
                    local_point_1.pose.position.y = point_1.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 오른쪽 얕은 경로 뽑기 (-) -----------------------------------//
                    local_point_2.pose.position.y = -narrow_offset;
                    local_point_2.pose.position.x = 2*narrow_radius*sin(narrow_alpha) + (lookAhead_distance - 2*narrow_radius*sin(narrow_alpha))*(j-50)/(point_num/3);
                    slope_2 = tf::createQuaternionFromRPY(0,0,M_PI/2);
                    local_point_2.pose.orientation.x = slope_2[0];
                    local_point_2.pose.orientation.y = slope_2[1];
                    local_point_2.pose.orientation.z = slope_2[2];
                    local_point_2.pose.orientation.w = slope_2[3];
                    tf::Vector3 point_2(local_point_2.pose.position.x,local_point_2.pose.position.y,0.0);
                    point_2 = rotation_ego2lookAhead*point_2;
                    local_point_2.pose.position.x = point_2.getX();
                    local_point_2.pose.position.y = point_2.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_3.pose.position.y = wide_offset;
                    local_point_3.pose.position.x = 2*wide_radius*sin(wide_alpha) + (lookAhead_distance - 2*wide_radius*sin(wide_alpha))*(j-50)/(point_num/3);
                    slope_3 = tf::createQuaternionFromRPY(0,0,M_PI/2);
                    local_point_3.pose.orientation.x = slope_3[0];
                    local_point_3.pose.orientation.y = slope_3[1];
                    local_point_3.pose.orientation.z = slope_3[2];
                    local_point_3.pose.orientation.w = slope_3[3];
                    tf::Vector3 point_3(local_point_3.pose.position.x,local_point_3.pose.position.y,0.0);
                    point_3 = rotation_ego2lookAhead*point_3;
                    local_point_3.pose.position.x = point_3.getX();
                    local_point_3.pose.position.y = point_3.getY();
                // -----------------------------------------------------------------------------------------//
                // --------------------------------- 왼쪽 얕은 경로 뽑기 (+) -----------------------------------//
                    local_point_4.pose.position.y = -wide_offset;
                    local_point_4.pose.position.x = 2*wide_radius*sin(wide_alpha) + (lookAhead_distance - 2*wide_radius*sin(wide_alpha))*(j-50)/(point_num/3);
                    slope_4 = tf::createQuaternionFromRPY(0,0,M_PI/2);
                    local_point_4.pose.orientation.x = slope_4[0];
                    local_point_4.pose.orientation.y = slope_4[1];
                    local_point_4.pose.orientation.z = slope_4[2];
                    local_point_4.pose.orientation.w = slope_4[3];
                    tf::Vector3 point_4(local_point_4.pose.position.x,local_point_4.pose.position.y,0.0);
                    point_4 = rotation_ego2lookAhead*point_4;
                    local_point_4.pose.position.x = point_4.getX();
                    local_point_4.pose.position.y = point_4.getY();
                // -----------------------------------------------------------------------------------------//
                
                }
                local_path_1.poses.push_back(local_point_1);
                local_path_2.poses.push_back(local_point_2);
                local_path_3.poses.push_back(local_point_3);
                local_path_4.poses.push_back(local_point_4);
                // printf("%d 번 돌았습니다. \n", j);
            }
            path_1 = local_path_1;
            path_2 = local_path_2;
            path_3 = local_path_3;
            path_4 = local_path_4;
            straight_path = local_path_st;

            check_collision();
        }
        
        void check_collision(){
            double cost_path[5];
            cost_path[0] = 0.0; cost_path[1] = 0; cost_path[2] = 0; cost_path[3] = 0; cost_path[4] = 0;
            // --- for collision check --- //
            // double radius_limit = vehicle_r+HurdleArray.obs_info.r;
            
            for(int i=0;i<point_num;i++){
                for(int j=0;j<HurdleArray.obs_info.size();j++){
                    double temp_distance_st = sqrt(pow(straight_path.poses[i].pose.position.x-HurdleArray.obs_info[j].x,2)+pow(straight_path.poses[i].pose.position.y-HurdleArray.obs_info[j].y,2));
                    if(temp_distance_st<=vehicle_r+HurdleArray.obs_info[j].r){
                        cost_path[4]+=100000000;
                        break;
                    }
                }            
            }
            for(int i=0;i<point_num;i++){
                for(int j=0;j<HurdleArray.obs_info.size();j++){
                    double temp_distance_st = sqrt(pow(path_1.poses[i].pose.position.x-HurdleArray.obs_info[j].x,2)+pow(path_1.poses[i].pose.position.y-HurdleArray.obs_info[j].y,2));
                    if(temp_distance_st<=vehicle_r+HurdleArray.obs_info[j].r){
                        cost_path[0]+=100000000;
                        break;
                    }
                }            
            }
            for(int i=0;i<point_num;i++){
                for(int j=0;j<HurdleArray.obs_info.size();j++){
                    double temp_distance_st = sqrt(pow(path_2.poses[i].pose.position.x-HurdleArray.obs_info[j].x,2)+pow(path_2.poses[i].pose.position.y-HurdleArray.obs_info[j].y,2));
                    if(temp_distance_st<=vehicle_r+HurdleArray.obs_info[j].r){
                        cost_path[1]+=100000000;
                        break;
                    }
                }            
            }
            for(int i=0;i<point_num;i++){
                for(int j=0;j<HurdleArray.obs_info.size();j++){
                    double temp_distance_st = sqrt(pow(path_3.poses[i].pose.position.x-HurdleArray.obs_info[j].x,2)+pow(path_3.poses[i].pose.position.y-HurdleArray.obs_info[j].y,2));
                    if(temp_distance_st<=vehicle_r+HurdleArray.obs_info[j].r){
                        cost_path[2]+=100000000;
                        break;
                    }
                }            
            }
            for(int i=0;i<point_num;i++){
                for(int j=0;j<HurdleArray.obs_info.size();j++){
                    double temp_distance_st = sqrt(pow(path_4.poses[i].pose.position.x-HurdleArray.obs_info[j].x,2)+pow(path_4.poses[i].pose.position.y-HurdleArray.obs_info[j].y,2));
                    if(temp_distance_st<=vehicle_r+HurdleArray.obs_info[j].r){
                        cost_path[3]+=100000000;
                        break;
                    }
                }            
            }
            // --------------------------- //
            // --- global path distance --- //
            geometry_msgs::PoseStamped path_last_pose;
            path_last_pose = path_1.poses.back();
            cost_path[0] += sqrt(pow(path_last_pose.pose.position.x-centerline_cost_pose_.pose.position.x,2)+pow(path_last_pose.pose.position.y-centerline_cost_pose_.pose.position.y,2));
            path_last_pose = path_2.poses.back();
            cost_path[1] += sqrt(pow(path_last_pose.pose.position.x-centerline_cost_pose_.pose.position.x,2)+pow(path_last_pose.pose.position.y-centerline_cost_pose_.pose.position.y,2));
            path_last_pose = path_3.poses.back();
            cost_path[2] += sqrt(pow(path_last_pose.pose.position.x-centerline_cost_pose_.pose.position.x,2)+pow(path_last_pose.pose.position.y-centerline_cost_pose_.pose.position.y,2));
            path_last_pose = path_4.poses.back();
            cost_path[3] += sqrt(pow(path_last_pose.pose.position.x-centerline_cost_pose_.pose.position.x,2)+pow(path_last_pose.pose.position.y-centerline_cost_pose_.pose.position.y,2));
            path_last_pose = straight_path.poses.back();
            cost_path[4] += sqrt(pow(path_last_pose.pose.position.x-centerline_cost_pose_.pose.position.x,2)+pow(path_last_pose.pose.position.y-centerline_cost_pose_.pose.position.y,2));
            // ---------------------------- //
            // ------------ Find Min Cost Idx -------------------//
            double min_cost = 99999999;
            proper_idx = 4;
            for(int i=0;i<5;i++){
                if(min_cost > cost_path[i]){
                    min_cost = cost_path[i];
                    proper_idx = i;
                }
            }
            publish_local_path();
        }

        void publish_local_path(){
            lookahead_.header.frame_id = "/ego";
            lookahead_.header.stamp = ros::Time::now();
            // lookahead.ns = "Lateral Control";
            lookahead_.id = 1;
            lookahead_.type = visualization_msgs::Marker::SPHERE;
            lookahead_.pose.position.x = lookAhead_Pose_.pose.position.x;
            lookahead_.pose.position.y = lookAhead_Pose_.pose.position.y;
            lookahead_.scale.x = 0.5;
            lookahead_.scale.y = 0.5;
            lookahead_.scale.z = 0.5;
            lookahead_.color.r = 0.0;
            lookahead_.color.g = 0.0;
            lookahead_.color.b = 1.0;
            lookahead_.color.a = 1.0;
            pub_lookahead_.publish(lookahead_);
            
            // proper_idx = 4;  // 임의로 proper idx를 4로 두어 직진을 보냄. 이전에 path를 보내지 못하는 오류 때문

            if(proper_idx == 0){
                pub_pose.publish(path_1);
                pub_candidate_1.publish(path_2);
                pub_candidate_2.publish(path_3);
                pub_candidate_3.publish(path_4);
                pub_candidate_4.publish(straight_path);
                ROS_INFO_STREAM("path_111");
            }
            else if(proper_idx == 1){
                pub_pose.publish(path_2);
                pub_candidate_1.publish(path_1);
                pub_candidate_2.publish(path_3);
                pub_candidate_3.publish(path_4);
                pub_candidate_4.publish(straight_path);
                ROS_INFO_STREAM("path_222");
            }
            else if(proper_idx == 2){
                pub_pose.publish(path_3);
                pub_candidate_1.publish(path_1);
                pub_candidate_2.publish(path_2);
                pub_candidate_3.publish(path_4);
                pub_candidate_4.publish(straight_path);
                ROS_INFO_STREAM("path_333");
            }
            else if(proper_idx == 3){
                pub_pose.publish(path_4);
                pub_candidate_1.publish(path_1);
                pub_candidate_2.publish(path_2);
                pub_candidate_3.publish(path_3);
                pub_candidate_4.publish(straight_path);
                ROS_INFO_STREAM("path_444");
            }
            else{
                pub_pose.publish(straight_path);
                pub_candidate_1.publish(path_1);
                pub_candidate_2.publish(path_2);
                pub_candidate_3.publish(path_3);
                pub_candidate_4.publish(path_4);
                ROS_INFO_STREAM("path_st");
            }
        }
        
        void Publish_Local(){
            // if( (execute_cal == false) ){
            find_ClosestWaypoint();
            ROS_WARN("PUBLISH");
            // }
        }
};


int main(int argc, char **argv){

    ros::init(argc,argv,"pub_path");    
    pointPub pub;
            
    ros::Rate rate(10);

    while(ros::ok()){
        pub.Publish_Local();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

