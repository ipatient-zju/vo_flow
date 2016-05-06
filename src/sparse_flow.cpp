#include <vo_flow/sparse_flow.h>

SparseFlow::SparseFlow():BaseFlow()
{
	ROS_INFO("Starting SparseFlow Class......");
	flow_pub = nh.advertise<vo_flow::OpticalFlow>("/vo_flow/sparse_flow/opt_flow",1);
	string image_topic;
	nh.param("image_topic",image_topic,string("/usb_cam/image_rect"));
	image_sub = nh.subscribe(image_topic.c_str(),1,&SparseFlow::imageCallback,this);

	nh.param("have_pixhawk",have_pixhawk,false);
	if(have_pixhawk == true)
		height_sub = nh.subscribe("/mavros/px4flow/ground_distance",1,&SparseFlow::heightCallback,this);
	else
		height_sub = nh.subscribe("/ultrasonic/ground_distance",1,&SparseFlow::heightCallback,this);

	nh.param("focus",focus,410);
	nh.param("point_num",point_num,36);  //16  25  36
	nh.param("err_qLevel",err_qLevel,10);

	height = 0;
	height_counter = 0;
	filter_method = 0;
	save_img = 0;
}

SparseFlow::~SparseFlow()
{
	ROS_INFO("Destroying SparseFlow Class...");
}

void SparseFlow::heightCallback(const sensor_msgs::Range::ConstPtr msg)
{
	if(msg->range > 0.3 && msg->range <3 && msg->range >(height-0.5) && msg->range<(height+0.5))
	{
		height = msg->range;
		height_counter=0;
	}
	else
	{
		height_counter++;
		if(height_counter ==8)
		{
			height_counter=0;
			height = msg->range;
		}
	}
}

void SparseFlow::imageCallback(const sensor_msgs::Image::ConstPtr msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    current_image = cv_ptr->image.clone();  // use clone or is a reference of orignal image
	display_image = current_image.clone();

	namedWindow("sparse_flow",CV_WINDOW_AUTOSIZE);
	vector<uchar> status;
	vector<float> err;
    int max_f = 1000;
	if(!previous_image.data || !current_image.data)
    {
        ROS_INFO("image is empty......");
        t_now = msg->header.stamp;
        t_last = t_now;
    }
    else
    {
		pre_points.clear();
		cur_points.clear();
		status.clear();
		err.clear();
        for (size_t i = 0; i < sqrt(point_num); i++)
        {
            for (size_t j = 0; j < sqrt(point_num); j++)
            {
				Point2f feature_point;
				float search_height = current_image.rows/sqrt(point_num);
                float search_width = current_image.cols/sqrt(point_num);
				feature_point.x = (int)(search_width*(i+0.5));
				feature_point.y = (int)(search_height*(j+0.5));
				pre_points.push_back(feature_point);

				line(display_image,Point(search_width*i+search_width,search_height*j+search_height),
					Point(search_width*i,search_height*j+search_height),Scalar(100),2);

				line(display_image,Point(search_width*i+search_width,search_height*j+search_height),
					Point(search_width*i+search_width,search_height*j),Scalar(100),2);
			}
		}

		calcOpticalFlowPyrLK(previous_image,current_image,pre_points,cur_points,status, err);

		int k = 0;
		for(size_t i = 0;i < cur_points.size();i++)
		{
			if((status[i]>0 ? (err[i]<err_qLevel ? 1:0):0) && ((abs(pre_points[i].x - cur_points[i].x) +
					abs(pre_points[i].y - cur_points[i].y)) > 2))
			{
				pre_points[k] = pre_points[i];
				cur_points[k] = cur_points[i];
				k++;
			}
		}
		pre_points.resize(k); //change size to k,k maybe less than pre_size,the item left will be lost
		cur_points.resize(k);

		for(size_t i = 0;i < cur_points.size();i++)
		{
			Point3i temp;
			temp.x = cur_points[i].x - pre_points[i].x;
			temp.y = cur_points[i].y - pre_points[i].y;
			temp.z = 0;
			point_vec.push_back(temp);

			circle(display_image,pre_points[i],2,Scalar(0),-1);
			line(display_image,pre_points[i], cur_points[i], Scalar(70),2);
			circle(display_image, cur_points[i],2,Scalar(0),-1);
		}

        t_now = msg->header.stamp;
        dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec));
        t_last = t_now;

        /*if(have_pixhawk == true)
            flow_msg.header.stamp = header.stamp;
        else*/
            flow_msg.header.stamp = t_now;

        Point3i flow_result;
		if(point_vec.size()!=0)
		{
        	if(filter_method == 0)
            	flow_result = weng_method(point_vec);
        	else
            	flow_result = like_ransac(point_vec);
			point_vec.clear();
		}
		else
		{
			flow_result.x = 0;
			flow_result.y = 0;
			flow_result.z = 0;
		}

        int where_cols = (flow_result.z/((int)sqrt(point_num)) + 0.5)*current_image.cols/sqrt(point_num);
        int where_rows = (flow_result.z%((int)sqrt(point_num)) + 0.5)*current_image.rows/sqrt(point_num);
        circle(display_image,Point2i(where_cols,where_rows),4,Scalar(0),-1);

		//cout<<"height: "<<height<<"dt: "<<dt<<endl;

        /***** calculate the velocity
        flow_msg.header.frame_id = "sparse_flow";
        flow_msg.ground_distance = height;
        flow_msg.flow_x = flow_result.x/dt;
        flow_msg.flow_y = flow_result.y/dt;
        flow_msg.velocity_x = -1.*(flow_msg.flow_x/focus*height);
        flow_msg.velocity_y = flow_msg.flow_y/focus*height;
        flow_msg.quality = 0;
        flow_pub.publish(flow_msg);

        *******/

        /***** calculate the distance ******/
        flow_msg.header.frame_id = "sparse_flow";
        flow_msg.ground_distance = height;
        flow_msg.flow_x = flow_result.x;
        flow_msg.flow_y = flow_result.y;
        flow_msg.velocity_x = -1.*(flow_msg.flow_x/focus*height);
        flow_msg.velocity_y = flow_msg.flow_y/focus*height;
        flow_msg.quality = 0;
        flow_pub.publish(flow_msg);



        //imshow("sparse_flow",display_image);
        //createTrackbar("focus:","sparse_flow",&focus,max_f,NULL);
        //createTrackbar("filter_method,0weng,1ransac:","sparse_flow",&filter_method,1,NULL);
		//createTrackbar("save_img:","sparse_flow",&save_img,1,NULL);
		if(waitKey(1) == 27  || save_img==1)
		{
			save_img=0;
			std::cout << "/* message */" << std::endl;
			int rand_id = (int)(1000.0*rand()/RAND_MAX+1.0);
			char filename[200];
			sprintf(filename, "/home/ycc/%d.png", rand_id);
			imwrite(filename,display_image);
		}
    }
    previous_image = current_image.clone();
}
