#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vo_flow/OpticalFlow.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <cmath>
using namespace std;
using namespace cv;

double dt = 0,vx_now,vy_now,px,py;
bool first_flag = true,first_flag_v=true;
int counter=0,filter=2,clear_flag=0;
ros::Time t_now,t_last;

deque<double> filter_vx(0,0),filter_vy(0,0);
deque<double> filter_vx1(0,0),filter_vy1(0,0);

ros::Publisher flow_path_pub,coordinate_pub,optitracker_path_pub,optitrack_pub;
nav_msgs::Path flow_path_msg,optitracker_path_msg;
geometry_msgs::PoseStamped pose_smsg,optitracker_msgs;
geometry_msgs::PointStamped coordinate_msg,velocity_msg;
float height_last;
std_msgs::Bool takeoff_msg;
bool first_take_flag=true;

ros::Time t_last_v,t_now_v;

void takeoff_callback(const std_msgs::Bool& msg)
{
	takeoff_msg=msg;
}

//caution ConstPtr& only!!!!!!!
void flow_callback(const vo_flow::OpticalFlow::ConstPtr flow_msg)
{
        if(first_flag == true)
        {
        	t_last = flow_msg->header.stamp;
			if(flow_msg->velocity_y>-5 && flow_msg->velocity_y<5)
				filter_vx.push_back(flow_msg->velocity_x);
			else
				filter_vx.push_back(0);
			if(flow_msg->velocity_x>-5 && flow_msg->velocity_x<5)
				filter_vy.push_back(flow_msg->velocity_y);
			else
				filter_vy.push_back(0);

        	counter++;
        	if(counter==5)first_flag = false;
			height_last = flow_msg->ground_distance;
        }
        else
        {
            t_now = flow_msg->header.stamp;
            dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec ));
        	t_last = t_now;

            filter_vx.pop_front();
            filter_vy.pop_front();

			if(flow_msg->velocity_y>-5 && flow_msg->velocity_y<5)
            	filter_vx.push_back(flow_msg->velocity_x);
			else
				filter_vx.push_back(0);
			if(flow_msg->velocity_x>-5 && flow_msg->velocity_x<5)
            	filter_vy.push_back(flow_msg->velocity_y);
			else
				filter_vy.push_back(0);

            if(filter==0)   // "0mean,1middle,2not"
            {
                vx_now = (filter_vx[0]+filter_vx[1]+filter_vx[2]+filter_vx[3]+filter_vx[4])/5.;
                vy_now = (filter_vy[0]+filter_vy[1]+filter_vy[2]+filter_vy[3]+filter_vy[4])/5.;
            }
            else if(filter==1)
            {
                filter_vx1.clear();filter_vx1.assign(filter_vx.begin(),filter_vx.end());
                sort(filter_vx1.begin(),filter_vx1.end());vx_now = filter_vx1[2];

                filter_vy1.clear();filter_vy1.assign(filter_vy.begin(),filter_vy.end());
                sort(filter_vy1.begin(),filter_vy1.end());vy_now = filter_vy1[2];
            }
            else
            {
                vx_now = filter_vx[4];
                vy_now = filter_vy[4];
            }

            px+=dt*vx_now;
            py+=dt*vy_now;
            if(clear_flag==1)
            {
                px=0;
                py=0;
            	flow_path_msg.poses.clear();
            }

			if(flow_msg->ground_distance > 1.2 && first_take_flag==true)
			{
				px=0;
            	py=0;
				first_take_flag=false;
			}

            flow_path_msg.header.stamp = ros::Time::now();
            flow_path_msg.header.frame_id = "map";
            pose_smsg.pose.position.x = px;
            pose_smsg.pose.position.y = py;

			if(flow_msg->ground_distance<3 && flow_msg->ground_distance>0.3)
			{
				pose_smsg.pose.position.z = flow_msg->ground_distance;
				height_last = flow_msg->ground_distance;
			}
			else
				pose_smsg.pose.position.z = height_last;

            flow_path_msg.poses.push_back(pose_smsg);
            flow_path_pub.publish(flow_path_msg);

            coordinate_msg.point.x = px;
            coordinate_msg.point.y = py;

			if(flow_msg->ground_distance<3 && flow_msg->ground_distance>0.3)
			{
				coordinate_msg.point.z = flow_msg->ground_distance;
				height_last = flow_msg->ground_distance;
			}
			else
				coordinate_msg.point.z = height_last;
            coordinate_pub.publish(coordinate_msg);
        }
        waitKey(1);
}

void optitracker_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	optitracker_path_msg.header.stamp = msg->header.stamp;
	optitracker_path_msg.header.frame_id = "map";
	optitracker_path_msg.poses.push_back(*msg);
	if(clear_flag ==1)optitracker_path_msg.poses.clear();
	optitracker_path_pub.publish(optitracker_path_msg);
	if(first_flag_v==true)
	{
		first_flag_v=false;
		t_now_v = msg->header.stamp;
		t_last_v = t_now_v;
		optitracker_msgs = *msg;
	}
	else
	{
		t_now_v = msg->header.stamp;
		dt = (double( t_now_v.sec + 1e-9 * t_now_v.nsec )) - (double( t_last_v.sec + 1e-9 * t_last_v.nsec ));
		t_last_v = t_now_v;


		velocity_msg.header.stamp = msg->header.stamp;
		velocity_msg.point.x = (msg->pose.position.x - optitracker_msgs.pose.position.x)/dt;
		velocity_msg.point.y = (msg->pose.position.y - optitracker_msgs.pose.position.y)/dt;
		optitrack_pub.publish(velocity_msg);
		optitracker_msgs = *msg;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"coordinate_flow_node");
    ros::NodeHandle nh("~");
    flow_path_pub = nh.advertise<nav_msgs::Path>("/flow_path",10);
	optitracker_path_pub = nh.advertise<nav_msgs::Path>("/optitracker_path",10);
    coordinate_pub = nh.advertise<geometry_msgs::PointStamped>("/coordinate_flow",10);
	optitrack_pub = nh.advertise<geometry_msgs::PointStamped>("/optitracker_velocity",10);

	string flow_topic;
	ros::NodeHandle nh_param("~");
	nh_param.param("flow_topic",flow_topic,string("/serial_flow_msg"));
	ros::Subscriber flow_sub = nh.subscribe(flow_topic.c_str(), 100 , flow_callback);
	ros::Subscriber takeoff_sub = nh.subscribe("/flag4takeoff", 1 , takeoff_callback);
	ros::Subscriber optitracker_sub = nh.subscribe("/true_pose", 100 , optitracker_callback);

	namedWindow("filter_tuning",WINDOW_NORMAL);
    createTrackbar( "clear:", "filter_tuning", &clear_flag, 1, NULL);
    createTrackbar( "filter:", "filter_tuning", &filter, 2, NULL);

    ros::spin();
    return 0;
}
