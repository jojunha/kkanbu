#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <kkanbu_sensor/object_info.h>


cv::Mat image;
ros::Publisher yolo_info;

static int count = 0;

void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{       
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("show", cv_ptr->image);
    image = cv_ptr->image;
    
    cv::waitKey(1);
}

void boundingCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    int size = boost::size(msg->bounding_boxes);
    for(int i = 0; i < size; i++){
        int xmin = msg->bounding_boxes[i].xmin;
        int ymin = msg->bounding_boxes[i].ymin;
        int xmax = msg->bounding_boxes[i].xmax;
        int ymax = msg->bounding_boxes[i].ymax;

        if((xmax - xmin)*(ymax - ymin) < 200){
            continue;
        }
        else{
            kkanbu_sensor::object_info object_info;

            object_info.latitude = 0;
            object_info.longitude = 0;
            object_info.class_name = msg->bounding_boxes[i].Class;
            
            std::string file_name = "yolo_image";
            file_name.append(std::to_string(count));
            file_name.append(".png");
            count +=1;
            std::cout << file_name << std::endl;
            
            cv::resize(image, image, cv::Size(512,512));
            cv::imwrite(file_name, image);

            object_info.file_name = file_name;

            yolo_info.publish(object_info);

        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_detection");
    ros::NodeHandle nh;

    ros::Subscriber image_raw = nh.subscribe("/camera/color/image_raw", 1, &imageCb);
    ros::Subscriber yolo_bounding = nh.subscribe("/darknet_ros/bounding_boxes", 1, &boundingCb);
    yolo_info = nh.advertise<kkanbu_sensor::object_info>("/yolo_info", 1);

    ros::spin();
    
    return 0;
}
