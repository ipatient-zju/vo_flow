#ifndef IMU_FLOW_FUSION_H_
#define IMU_FLOW_FUSION_H_

#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <vo_flow/OpticalFlow.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <std_msgs/Bool.h>

using namespace std;
using namespace cv;

class ImuFlowFusion
{
public:
    ros::NodeHandle nh;
    ros::Subscriber flow_sub,imu_sub;
    ros::Publisher fusion_path_pub,fusion_velocity_pub;

    geometry_msgs::PointStamped velocity_msg;

    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_msgs;

    Mat A,B,C,Q,R,P,U,X,X_priori,Y,Rotation_b2w_a,Rotation_b2w_v;
    bool flow_ready,first_flag;
    int first_flag_counter;
    double flow_vx,flow_vy,dt,height_last,height,q[4],a[3],yaw,bais_a[3];
    ros::Time t_last,t_now;

    ImuFlowFusion();
    ~ImuFlowFusion();
    void flowCallback(const vo_flow::OpticalFlow::ConstPtr msg);
    void q2rotation(double q[]);
    void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
};

#endif
