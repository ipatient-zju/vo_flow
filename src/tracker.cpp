#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include  "sensor_msgs/Range.h"
#include <iostream>

#define FOCUS 100
#define out_threshold 45

using namespace std;
using namespace cv;

geometry_msgs::Twist tracker_cmd;
ros::Publisher tracker_publisher;
float height=0;
float error_x=0,error_y=0,out_x=0,out_y=0,error_x_last=0,error_y_last=0;
int tracker_counter=0;
int p=150,d=30;

void height_callback(const sensor_msgs::Range::ConstPtr msg)
{
        height = msg -> range;
}

void point_callback(const geometry_msgs::Point::ConstPtr msg)
{
        if(msg->z > 0)
        {
                error_x = (height/FOCUS)*(msg->x-320);
                error_y = (height/FOCUS)*(240-msg->y);
                out_x = p /10. * error_x + d * (error_x - error_x_last);
                out_y = p /10. * error_y + d * (error_y - error_y_last);
                cout << error_x <<" "<< error_y << "   "<< out_x <<" "<<  out_y  <<endl;
                if(out_x>out_threshold)out_x=out_threshold;
                if(out_x<-1*out_threshold)out_x=-1*out_threshold;
                if(out_y>out_threshold)out_y=out_threshold;
                if(out_y<-1*out_threshold)out_y=-1*out_threshold;
                tracker_cmd.linear.x = int(out_x);
                tracker_cmd.linear.y = int(out_y);
                tracker_cmd.angular.x = 1;
                tracker_publisher.publish(tracker_cmd);
                error_x_last = error_x;
                error_y_last = error_y;
        }
        else
        {
                tracker_cmd.linear.x = 0;
                tracker_cmd.linear.y = 0;
                tracker_cmd.angular.x = 0;
                tracker_publisher.publish(tracker_cmd);
        }

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"multi_tracker_node");
	ros::NodeHandle n;
	ros::Subscriber  height_sub = n.subscribe("/mavros/px4flow/ground_distance",10,height_callback);
       ros::Subscriber  point_sub = n.subscribe("/position_to_land_topic",10,point_callback);
       tracker_publisher = n.advertise<geometry_msgs::Twist>("/tracker_cmd",10);

      namedWindow("tracker_parameter_tuning",WINDOW_NORMAL);  //WINDOW_NORMAL   CV_WINDOW_AUTOSIZE
      moveWindow("tracker_parameter_tuning",240,180);
      createTrackbar( " p:", "tracker_parameter_tuning", &p, 150, NULL );
      createTrackbar( " d:", "tracker_parameter_tuning", &d, 100, NULL );
             ros::Rate loop_rate(250);
             while(ros::ok())
             {
                 tracker_counter++;
                 if (tracker_counter>150)
                 {
                        tracker_cmd.linear.x = 0;
                        tracker_cmd.linear.y = 0;
                        //tracker_publisher.publish(tracker_cmd);
                 }
                 ros::spinOnce();
                 loop_rate.sleep();
                 waitKey(1);
              }
	return 0;
}
