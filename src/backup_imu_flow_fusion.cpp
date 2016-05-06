#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <px_comm/OpticalFlow.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <px4flow_hover/fusion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <deque>
#include <cmath>
using namespace std;
using namespace cv;
using namespace message_filters;

#define G 9.8

Mat Q = (Mat_<double>(6,6) << 64., 0, 0, 0, 0, 0,
                              0, 64., 0, 0, 0, 0,
                              0, 0, 36., 0, 0, 0,
                              0, 0, 0, 36., 0, 0,
                              0, 0, 0, 0, 64., 0,
                              0, 0, 0, 0, 0, 64.
                           );

Mat R = (Mat_<double>(4,4) << 36., 0, 0, 0,
                              0, 36., 0, 0,
                              0, 0, 64., 0,
                              0, 0, 0, 64.
                           );

Mat P = (Mat_<double>(6,6) << 4., 0, 0, 0, 0, 0,
                              0, 4., 0, 0, 0, 0,
                              0, 0, 4., 0, 0, 0,
                              0, 0, 0, 4., 0, 0,
                              0, 0, 0, 0, 4., 0,
                              0, 0, 0, 0, 0, 4.
                            );

Mat A = (Mat_<double>(6,6) << 1., 0, 0., 0, 0, 0,
                              0, 1., 0, 0., 0, 0,
                              0, 0, 1., 0, 0., 0,
                              0, 0, 0, 1., 0, 0.,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0
                            );

Mat B = (Mat_<double>(6,2) << 0, 0,
                              0, 0,
                              0, 0,
                              0, 0,
                              G, 0,
                              0, G
                            );

Mat U = (Mat_<double>(2,1) << 0,
                              0
                            );

Mat C = (Mat_<double>(4,6) << 0, 0, 1., 0, 0, 0,
                              0, 0, 0, 1., 0, 0,
                              0, 0, 0, 0, 1., 0,
                              0, 0, 0, 0, 0, 1.
                            );

Mat X = (Mat_<double>(6,1) << 0,
                              0,
                              0,
                              0,
                              0,
                              0
                            );

Mat X_priori = (Mat_<double>(6,1) << 0,
                                     0,
                                     0,
                                     0,
                                     0,
                                     0
                                   );

Mat Y = (Mat_<double>(4,1) << 0,
                              0,
                              0,
                              0
                            );


double dt = 0,vx_now,vy_now,ax_now,ay_now;
bool first_flag = true;
int counter=0,filter=0,useeuler=1;
ros::Time t_now,t_last;

deque<double> filter_vx(0,0),filter_vy(0,0),filter_ax(0,0),filter_ay(0,0);
deque<double> filter_vx1(0,0),filter_vy1(0,0),filter_ax1(0,0),filter_ay1(0,0);

Mat P_priori(6,6,CV_64FC1);
Mat II = Mat::eye(6, 6, CV_64FC1);
Mat K(6,4,CV_64FC1);

//Mat Q,R,P,A,B,U,C,X,X_priori,Y,K

ros::Publisher data_fusion_pub,path_pub,coordinate_pub;
px4flow_hover::fusion data;
nav_msgs::Path path_msg;
geometry_msgs::PoseStamped pose_smsg;
geometry_msgs::PointStamped coordinate_msg;
double pitch,roll,yaw,q0,q1,q2,q3;

//caution ConstPtr& only!!!!!!!
void callback(const px_comm::OpticalFlow::ConstPtr flow_msg, const sensor_msgs::Imu::ConstPtr imu_msg)
{
        if(first_flag == true)
        {
                t_last = flow_msg->header.stamp;
                filter_vx.push_back(-1.0*flow_msg->velocity_x);
                filter_vy.push_back(flow_msg->velocity_y);
                filter_ax.push_back(imu_euler_msg->ax);
                filter_ay.push_back(imu_euler_msg->ay);
                counter++;
                if(counter==5)
                {
                        first_flag = false;
                }
        }
        else
        {
                t_now = flow_msg->header.stamp;
                dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec ));
                t_last = t_now;
                A.at<double>(0,2) = dt;
                A.at<double>(1,3) = dt;
                A.at<double>(2,4) = dt;
                A.at<double>(3,5) = dt;
                A.at<double>() = 0.5*dt*dt;
                A.at<double>() = 0.5*dt*dt;

                q0 = imu_msg.orientation.x;
                q1 = imu_msg.orientation.y;
                q2 = imu_msg.orientation.z;
                q3 = imu_msg.orientation.w;

                pitch = arcsin(2.*(q3*q1 - q2*q0));
                roll = arctan(2.*(q3*q0 + q1*q2)/(1-2.*(q0*q0+q1*q1)));
                yaw = arctan(2.*(q3*q0 + q1*q2)/(1-2.*(q0*q0+q1*q1)));

                U.at<double>(0,0) = tan(pitch);
                U.at<double>(1,0) = tan(-1.*roll);

                filter_vx.pop_front();
                filter_vy.pop_front();
                filter_ax.pop_front();
                filter_ay.pop_front();

                filter_vx.push_back(-1.0*flow_msg->velocity_x);
                filter_vy.push_back(flow_msg->velocity_y);
                filter_ax.push_back(imu_euler_msg->linear_acceleration.x);
                filter_ay.push_back(imu_euler_msg->linear_acceleration.y);


                if(filter==0)   // "0mean,1middle,2not"
                {
                        vx_now = (filter_vx[0]+filter_vx[1]+filter_vx[2]+filter_vx[3]+filter_vx[4])/5.;
                        vy_now = (filter_vy[0]+filter_vy[1]+filter_vy[2]+filter_vy[3]+filter_vy[4])/5.;
                        ax_now = (filter_ax[0]+filter_ax[1]+filter_ax[2]+filter_ax[3]+filter_ax[4])/5.;
                        ay_now = (filter_ay[0]+filter_ay[1]+filter_ay[2]+filter_ay[3]+filter_ay[4])/5.;
                }
                else if(filter==1)
                {
                        filter_vx1.clear();filter_vx1.assign(filter_vx.begin(),filter_vx.end());
                        sort(filter_vx1.begin(),filter_vx1.end());vx_now = filter_vx1[2];

                        filter_vy1.clear();filter_vy1.assign(filter_vy.begin(),filter_vy.end());
                        sort(filter_vy1.begin(),filter_vy1.end());vy_now = filter_vy1[2];

                        filter_ax1.clear();filter_ax1.assign(filter_ax.begin(),filter_ax.end());
                        sort(filter_ax1.begin(),filter_ax1.end());ax_now = filter_ax1[2];

                        filter_ay1.clear();filter_ay1.assign(filter_ay.begin(),filter_ay.end());
                        sort(filter_ay1.begin(),filter_ay1.end());ay_now = filter_ay1[2];
                }
                else
                {
                        vx_now = filter_vx[4];
                        vy_now = filter_vy[4];
                        ax_now = filter_ax[4];
                        ay_now = filter_ay[4];
                }

                Y.at<double>(0,0) = vx_now;
                Y.at<double>(1,0) = vy_now;
                Y.at<double>(2,0) = ax_now;
                Y.at<double>(3,0) = ay_now;

                //"0noteuler,1euler"
                X_priori = A*X + useeuler*B*U;
                P_priori = A*P*A.t() + Q;


                K = P_priori*C.t()*((C*P_priori*C.t()+R).inv());  //DECOMP_LU   DECOMP_CHOLESKY
                X = (X_priori + K*(Y-C*X_priori));

                P = (II - K*C)*P_priori;

                data.px = X.at<double>(0,0);
                data.py = X.at<double>(1,0);
                data.vx = X.at<double>(2,0);
                data.vy = X.at<double>(3,0);
                data.ax = X.at<double>(4,0);
                data.ay = X.at<double>(5,0);
                data_fusion_pub.publish(data);

                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";
                pose_smsg.pose.position.x = X.at<double>(0,0);
                pose_smsg.pose.position.y = X.at<double>(1,0);
                pose_smsg.pose.position.z = flow_msg->ground_distance;
                path_msg.poses.push_back(pose_smsg);
                path_pub.publish(path_msg);

                coordinate_msg.point.x = data.px;
                coordinate_msg.point.y = data.py;
                coordinate_msg.point.z = flow_msg->ground_distance;
                coordinate_pub.publish(coordinate_msg);
        }
        waitKey(1);
}

int main(int argc, char **argv)
{
        ros::init(argc,argv,"imu_flow_fusion_node");
        ros::NodeHandle nh("~");
        data_fusion_pub = nh.advertise<px4flow_hover::fusion>("/fusion_data",100);
        path_pub = nh.advertise<nav_msgs::Path>("/fusion_path",10);
        coordinate_pub = nh.advertise<geometry_msgs::PointStamped>("/coordinate_imu_flow",10);

        message_filters::Subscriber<px_comm::OpticalFlow> flow_sub(nh, "/serial_flow_msg", 100);
        message_filters::Subscriber<sensor_msgs::Imu> imu_euler_sub(nh, "/mavros/imu/data", 100);

        typedef sync_policies::ApproximateTime<px_comm::OpticalFlow, imu_ultrasonic_data::imu_euler> MySyncPolicy;
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), flow_sub, imu_euler_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2));

        namedWindow("filter_tuning",WINDOW_NORMAL);           //WINDOW_NORMAL   CV_WINDOW_AUTOSIZE
        createTrackbar( "0mean,1middle,2not", "filter_tuning", &filter, 2, NULL );
        createTrackbar( "0noteuler,1euler", "filter_tuning", &useeuler, 1, NULL );

        ros::spin();
        return 0;
}
