#include"kkanbu_merge.hpp"

#define PI 3.141592

merge_circle::merge_circle(){
    markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_lidar_circle",1000);
    camera_markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_camera_circle",1000);
    merge_markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("/viz_merge_circle",1000);
    sub_lidar_circle = n.subscribe("/obstacles", 1000, &merge_circle::lidarCallback, this);
    sub_camera_circle = n.subscribe("C_Obstacles",1000, &merge_circle::CameraCallback, this);
    
}
merge_circle::~merge_circle(){

}

visualization_msgs::Marker merge_circle::markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id, double color){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.ns = "basic_shapes";
        marker.id = _id;
        marker.pose.position.x = point.pose.position.x;
        marker.pose.position.y = point.pose.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scale*2; //임의의 비행금지구역 반경 (x,y 같게)
        marker.scale.y = scale*2;
        marker.scale.z = scale_z; //비행금지구역 높이는 무제한

        marker.color.r = 1.0f;
        marker.color.g = color;
        marker.color.b = 0.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(0.1); //마커 지속시간
        marker.header.frame_id = "/map";
        return marker;
    }
    
void merge_circle::viz_lidar_circle(){
    D_WPArray.markers.clear();
    num = 0;
    for (auto& r : lidar_circle.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        D_WPArray.markers.push_back(markergenerator(wp,r.radius,0.1,num,1.0));
        num++;
    }
    markerArrayPub.publish(D_WPArray);
}

void merge_circle::viz_merge_circle(){
    D_WPArray.markers.clear();
    for (auto& r : merge_circles.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        D_WPArray.markers.push_back(markergenerator(wp,r.radius,0.1,num,0.5));
        num++;
    }
    merge_markerArrayPub.publish(D_WPArray);
}

void merge_circle::viz_camera_circle(){
    D_WPArray.markers.clear();
    for(auto& r : camera_circle.circles){
        wp.pose.position.x = r.center.x;
        wp.pose.position.y = r.center.y;
        D_WPArray.markers.push_back(markergenerator(wp,r.radius,0.1,num,0.0));
        num++;
    }
    camera_markerArrayPub.publish(D_WPArray);
}

void merge_circle::lidarCallback(kkanbu_msgs::Obstacles _lidar_circle ){
    lidar_circle = _lidar_circle;
}

void merge_circle::CameraCallback(kkanbu_msgs::C_Obstacles _camera_obstacle){
    camera_circle.circles.clear();
    for(auto& r: _camera_obstacle.camera_obstacle){
        kkanbu_msgs::CircleObstacle c;
        if(r.angle > 42){
            std::cout << r.angle <<'\n';
        }
        if(r.angle < -42){
            std::cout << r.angle <<'\n';
        }
        c.radius = r.width/2;
        c.center.x = r.distance;
        c.center.y = -r.distance*tan(r.angle*PI/180);

//         if(r.angle <= 0){
//             double d = (r.distance*1/cos(abs(r.angle*180/PI)));
//  //           d = d + r.width/2;
//             c.radius = r.width/2;
//             c.center.x = - d*sin(abs(r.angle));
//             c.center.y = r.distance;
//         }
//         else{
//             double d = (r.distance*1/cos(abs(r.angle*180/PI)));
//             //d = d + r.width/2;
//             c.radius = r.width/2;
//             c.center.x = d*sin(abs(r.angle));
//             c.center.y = r.distance;
//         }
        camera_circle.circles.push_back(c);
    }
}

void merge_circle::merge(){
    merge_circles.circles.clear();
    kkanbu_msgs::CircleObstacle tmp;
    for (auto& r : lidar_circle.circles){
        for(auto & c_r : camera_circle.circles){
            double distance = pow(pow(r.center.x-c_r.center.x,2)+pow(r.center.y-c_r.center.y,2),0.5);
            if(distance+c_r.radius <= r.radius){
                merge_circles.circles.push_back(r);
            }
            else if(distance+r.radius <= c_r.radius){
                merge_circles.circles.push_back(c_r);
            }
            else if(distance < r.radius + c_r.radius){
                
                // double rate; rate = c_r.radius/r.radius;
                // tmp.center.x = (c_r.center.x + rate*r.center.x)/(1+rate);
                // tmp.center.y = (c_r.center.y + rate*r.center.y)/(1+rate);
                // double l_radius; double c_radius;
                // l_radius = (distance/(1+rate))+r.radius ; c_radius = (distance*rate/(1+rate)) + c_r.radius;
                // if(l_radius >= c_radius){
                // tmp.radius = l_radius;
                // }
                // else{
                // tmp.radius = c_radius;
                // }
                // merge_circles.circles.push_back(tmp);

                Point tmp_v; Point tmp_l; Point tmp_c;
                tmp_v.x = c_r.center.x - r.center.x; // lidar에서 camera 방향
                tmp_v.y = c_r.center.y - r.center.y;
                tmp_v = tmp_v.normalized();
                
                tmp_c.x = c_r.center.x + c_r.radius*tmp_v.x;
                tmp_c.y = c_r.center.y + c_r.radius*tmp_v.y;

                tmp_l.x = r.center.x + -r.radius*tmp_v.x;
                tmp_l.y = r.center.y + -r.radius*tmp_v.y;  
                
                tmp.radius = (distance+ r.radius + c_r.radius)/2; 
                tmp.center.x = (tmp_c.x+tmp_l.x)/2;
                tmp.center.y = (tmp_c.y+tmp_l.y)/2;

                merge_circles.circles.push_back(tmp);
           }
        }
    }
    for (auto& c_r : camera_circle.circles){
        bool ex = false;
        for(auto & r : merge_circles.circles){
            double distance = pow(pow(r.center.x-c_r.center.x,2)+pow(r.center.y-c_r.center.y,2),0.5);
            if(distance < r.radius + c_r.radius)
                ex = true;
        }
        if(!ex){
            merge_circles.circles.push_back(c_r);
        }
    }
    for (auto& l_r : lidar_circle.circles){
        bool ex = false;
        for(auto & r : merge_circles.circles){
            double distance = pow(pow(r.center.x-l_r.center.x,2)+pow(r.center.y-l_r.center.y,2),0.5);
            if(distance < r.radius + l_r.radius)
                ex = true;
        }
        if(!ex){
            merge_circles.circles.push_back(l_r);
        }
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "merge_sensor");
    merge_circle mer;
    ros::Rate r(1000);
    while(ros::ok()){
        mer.merge();
        mer.viz_lidar_circle();
        mer.viz_camera_circle();
        mer.viz_merge_circle();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

// geometry_msgs/Point center      # Central point [m]
// geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
// float64 radius                  # Radius with added margin [m]
// float64 true_radius             # True measured radius [m]
// float64 angle			# Angle of Center [degree]
// float64 c_distance		# Distance from Center [m]
// float64 min_distance		# Min_Distance [m]

// Header header

// obstacle_detector/SegmentObstacle[] segments
// obstacle_detector/CircleObstacle[] circles

// if(distance < r.radius + c_r.radius){
//     double rate; rate = c_r.radius/r.radius;
//     tmp.center.x = (c_r.center.x + rate*r.center.x)/(1+rate);
//     tmp.center.y = (c_r.center.y + rate*r.center.y)/(1+rate);
//     double l_radius; double c_radius;
//     l_radius = (distance/(1+rate))+r.radius ; c_radius = (distance*rate/(1+rate)) + c_r.radius;
//     if(l_radius >= c_radius){
//         tmp.radius = l_radius;
//     }
//     else{
//         tmp.radius = c_radius;
//     }
//     merge_circles.circles.push_back(tmp);
// }
