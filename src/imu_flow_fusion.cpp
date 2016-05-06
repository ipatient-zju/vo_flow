#include <vo_flow/imu_flow_fusion.h>

ImuFlowFusion::ImuFlowFusion():nh("~")
{
    ROS_INFO("Starting ImuFlowFusion Node......");
    string flow_topic;
    nh.param("flow_topic",flow_topic,string("/vo_flow/sparse_flow/opt_flow"));
    flow_sub = nh.subscribe(flow_topic.c_str(),1,&ImuFlowFusion::flowCallback,this);

    imu_sub = nh.subscribe("/mavros/imu/data",1,&ImuFlowFusion::imuCallback,this);

    fusion_path_pub = nh.advertise<nav_msgs::Path>("/fusion_path",10);
    fusion_velocity_pub = nh.advertise<geometry_msgs::PointStamped>("/fusion_velocity",10);

    flow_ready = false;
    first_flag = true;
    first_flag_counter=0;
    flow_vx = 0;
    flow_vy = 0;

    Q = (Mat_<double>(4,4) << 64,0,0,0,0,64,0,0,0,0,64,0,0,0,0,64);

    R = (Mat_<double>(2,2) << 360000,0,0,360000);

    P = (Mat_<double>(4,4) << 4,0,0,0,0,4,0,0,0,0,4,0,0,0,0,4);

    A = (Mat_<double>(4,4) << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1); //t

    B = (Mat_<double>(4,2) << 0,0,0,0,0,0,0,0); //t*t

    U = (Mat_<double>(2,1) << 0,0);

    C = (Mat_<double>(2,4) << 0,0,1,0,0,0,0,1);

    X = (Mat_<double>(4,1) << 0,0,0,0);

    X_priori = (Mat_<double>(4,1) << 0,0,0,0);

    Y = (Mat_<double>(2,1) << 0,0);

    Rotation_b2w_a = (Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1);
    Rotation_b2w_v = (Mat_<double>(2,2) << 1,0,0,1);
}

ImuFlowFusion::~ImuFlowFusion()
{
    ROS_INFO("Destroying ImuFlowFusion.....");
}

void ImuFlowFusion::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
{
    if(first_flag == true)
    {
        t_last = msg->header.stamp;
        first_flag_counter++;
        if(first_flag_counter>10)
        {
            first_flag_counter=0;
            first_flag = false;
            bais_a[0] = msg->linear_acceleration.x;
            bais_a[1] = msg->linear_acceleration.y;
            bais_a[2] = msg->linear_acceleration.z;
        }
        std::cout << "/* message */"<<first_flag_counter << std::endl;
    }
    else
    {
        t_now = msg->header.stamp;
        dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec ));
        t_last = t_now;

        q[1] = msg->orientation.x;
        q[2] = msg->orientation.y;
        q[3] = msg->orientation.z;
        q[0] = msg->orientation.w;

        q2rotation(q);

        //a[0] = msg->linear_acceleration.x-bais_a[0];
        //a[1] = msg->linear_acceleration.y-bais_a[1];
        //a[2] = msg->linear_acceleration.z-bais_a[2];

        a[0] = msg->linear_acceleration.x;
        a[1] = msg->linear_acceleration.y;
        a[2] = msg->linear_acceleration.z;

        Mat a_body = (Mat_<double>(3,1) << a[0],a[1],a[2]);
        Mat a_inertial = Rotation_b2w_a*a_body;
        //R*v:acceleration relative to frame inertial
        std::cout << "a_inertial:"<<a_inertial << std::endl;
        U = (Mat_<double>(2,1) << a_inertial.at<double>(0,0),a_inertial.at<double>(1,0));

        A.at<double>(0,2) = dt;
        A.at<double>(1,3) = dt;

        B.at<double>(0,0) = dt*dt*0.5;
        B.at<double>(1,1) = dt*dt*0.5;
        B.at<double>(2,0) = dt;
        B.at<double>(3,1) = dt;

        X_priori = A*X + B*U;//state euqations
        Mat P_priori = A*P*A.t() + Q;//
        Mat II = Mat::eye(4, 4, CV_64FC1);

        //flow_ready=false;

        if(flow_ready==true)
        {
            flow_ready=false;
            Mat v_body = (Mat_<double>(2,1) << flow_vx,flow_vy);
            Mat v_inertial = Rotation_b2w_v*v_body;
            // velocity relative to frame inertial

            Y =(Mat_<double>(2,1) << v_inertial.at<double>(0,0),-1*v_inertial.at<double>(1,0));
            std::cout << v_body<<"Y:"<<Y << std::endl;

            Y = v_body.clone();
            //C: obsevation matrix
            Mat K = P_priori*C.t()*((C*P_priori*C.t()+R).inv());  //DECOMP_LU   DECOMP_CHOLESKY
            X = (X_priori + K*(Y-C*X_priori));
            P = (II - K*C)*P_priori;
            std::cout << "K"<<K << std::endl;
            path_msg.header.stamp = msg->header.stamp;
            path_msg.header.frame_id = "map";
            pose_msgs.pose.position.x = X.at<double>(0,0);
            pose_msgs.pose.position.y = X.at<double>(1,0);
            pose_msgs.pose.position.z = height;
            path_msg.poses.push_back(pose_msgs);
            //std::cout << X.at<double>(0,0)<<"haha"<<X.at<double>(1,0) << std::endl;
            fusion_path_pub.publish(path_msg);

            velocity_msg.point.x =  X.at<double>(2,0);
            velocity_msg.point.y =  X.at<double>(3,0);
            velocity_msg.header.stamp = msg->header.stamp;
            velocity_msg.header.frame_id = "map";
            fusion_velocity_pub.publish(velocity_msg);

            //std::cout << "ready" << std::endl;
        }
        else
        {
            X = X_priori.clone();
            P = P_priori.clone();

            path_msg.header.stamp = msg->header.stamp;
            path_msg.header.frame_id = "map";
            pose_msgs.pose.position.x = X.at<double>(0,0);
            pose_msgs.pose.position.y = X.at<double>(1,0);
            pose_msgs.pose.position.z = height;
            path_msg.poses.push_back(pose_msgs);
            fusion_path_pub.publish(path_msg);

            velocity_msg.point.x =  X.at<double>(2,0);
            velocity_msg.point.y =  X.at<double>(3,0);
            velocity_msg.header.stamp = msg->header.stamp;
            velocity_msg.header.frame_id = "map";
            fusion_velocity_pub.publish(velocity_msg);

            //std::cout << "not ready" << std::endl;
        }
    }
}
//solve the quantion to euler
void ImuFlowFusion::q2rotation(double q[])
{
    double q00 = q[0]*q[0];   double q01 = q[0]*q[1];   double q02 = q[0]*q[2];    double q03 = q[0]*q[3];
                              double q11 = q[1]*q[1];   double q12 = q[1]*q[2];    double q13 = q[1]*q[3];
                                                        double q22 = q[2]*q[2];    double q23 = q[2]*q[3];
                                                                                   double q33 = q[3]*q[3];

    Rotation_b2w_a = (Mat_<double>(3,3) << q00+q11-q22-q33,    2*(q12-q03),    2*(q13+q02),
                                           2*(q12+q03),     q00-q11+q22-q33,   2*(q23-q01),
                                           2*(q13-q02),     2*(q23+q01),    q00-q11-q22+q33);

    yaw = atan( 2*(q03 + q12)/(1-2*(q33+q22)) );
    std::cout << "yaw"<<yaw*180/3.14 << std::endl;

    Rotation_b2w_v = (Mat_<double>(2,2) << cos(yaw),-1*sin(yaw),sin(yaw),cos(yaw));
}

void ImuFlowFusion::flowCallback(const vo_flow::OpticalFlow::ConstPtr msg)
{
    flow_ready = true;
    height = msg->ground_distance;
    flow_vx = msg->velocity_x;
    flow_vy = msg->velocity_y;
}
