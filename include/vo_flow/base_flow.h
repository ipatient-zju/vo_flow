#ifndef BASE_FLOW_H_
#define BASE_FLOW_H_

#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <vo_flow/OpticalFlow.h>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

class BaseFlow
{
public:
    ros::NodeHandle nh;
    ros::Subscriber image_sub,height_sub,imu_sub;
    ros::Publisher flow_pub;

    Mat previous_image,current_image,display_image;

    vo_flow::OpticalFlow flow_msg;
    int focus,filter_method,save_img;
    ros::Time t_now,t_last;
    double dt,height;
        int height_counter;

    double q[4],a[3];

    bool have_pixhawk;

    Point3i weng_method(vector<Point3i> vec);
    Point3i like_ransac(vector<Point3i> vec);

    BaseFlow();
    ~BaseFlow();
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr msg)=0;
    virtual void heightCallback(const sensor_msgs::Range::ConstPtr msg)=0;
    //virtual void imuCallback(const sensor_msgs::Imu::ConstPtr msg)=0;
};

#endif
