#include <vo_flow/template_match_flow.h>

TemplateMatchFlow::TemplateMatchFlow():BaseFlow()
{
    ROS_INFO("Starting TemplateMatchFlow Class......");
    flow_pub = nh.advertise<vo_flow::OpticalFlow>("/vo_flow/template_match/opt_flow",1);
    string image_topic;
    nh.param("image_topic",image_topic,string("/usb_cam/image_rect"));
    image_sub = nh.subscribe(image_topic.c_str(),1,&TemplateMatchFlow::imageCallback,this);

    nh.param("have_pixhawk",have_pixhawk,false);
    if(have_pixhawk == true)
        height_sub = nh.subscribe("/mavros/px4flow/ground_distance",1,&TemplateMatchFlow::heightCallback,this);
    else
        height_sub = nh.subscribe("/ultrasonic/ground_distance",1,&TemplateMatchFlow::heightCallback,this);

    nh.param("focus",focus,654);
    nh.param("patch_num",patch_num,36);  //16  25  36
    nh.param("divide",divide,5);  //3 4 5
    height = 0;
    height_counter=0;
    match_method = 0;
    filter_method = 0;
    save_img = 0;
}

TemplateMatchFlow::~TemplateMatchFlow()
{
    ROS_INFO("Destroying TemplateMatchFlow Class......");
}

void TemplateMatchFlow::heightCallback(const sensor_msgs::Range::ConstPtr msg)
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

void TemplateMatchFlow::imageCallback(const sensor_msgs::Image::ConstPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
    current_image = cv_ptr->image.clone();  // use clone or is a reference of orignal image
    display_image = current_image.clone();

    int max_Trackbar = 5;
    int max_f = 1000;

    namedWindow("template_flow",CV_WINDOW_AUTOSIZE);

    string trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";

    if(!previous_image.data || !current_image.data)
    {
        ROS_INFO("image is empty......");
        t_now = msg->header.stamp;
        t_last = t_now;
    }
    else
    {
        for (size_t i = 0; i < sqrt(patch_num); i++)
        {
            for (size_t j = 0; j < sqrt(patch_num); j++)
            {
                int search_height = current_image.rows/sqrt(patch_num);
                int search_width = current_image.cols/sqrt(patch_num);
                int target_height = search_height/divide;
                int target_width = search_width/divide;

                Rect rect_sea(search_width*i,search_height*j,search_width,search_height);
                Mat search_image = current_image(rect_sea);

                Rect rect_tar(search_width*i+search_width/2-target_width/2,search_height*j+search_height/2-target_height/2,
                    target_width,target_height);
                Mat target_image = previous_image(rect_tar);

                /// Create the result matrix
                int result_cols = search_width - target_width + 1;
                int result_rows = search_height - target_height + 1;
                Mat result(result_rows, result_cols, CV_32FC1);

                /// Do the Matching and Normalize
                matchTemplate(search_image, target_image, result, match_method);
                normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

                // Localizing the best match with minMaxLoc
                double minVal; double maxVal; Point minLoc; Point maxLoc;
                Point matchLoc;

                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

                // For SQDIFF and SQDIFF_NORMED, the best matches are lower values.
                // For all the other methods, the higher the better
                if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
                    matchLoc = minLoc;
                else
                    matchLoc = maxLoc;

                //std::cout << matchLoc.x<<"  "<<matchLoc.y << std::endl;
                matchLoc.x = matchLoc.x +  search_width*i;
                matchLoc.y = matchLoc.y +  search_height*j;

                rectangle(display_image, matchLoc, Point( matchLoc.x + target_image.cols ,
                    matchLoc.y + target_image.rows), Scalar::all(0), 2, 8, 0 );

                line(display_image,Point(matchLoc.x + target_image.cols/2, matchLoc.y + target_image.rows/2),
                    Point(search_width*i+search_width/2,search_height*j+search_height/2),Scalar(70),2);

                line(display_image,Point(search_width*i+search_width,search_height*j+search_height),
                    Point(search_width*i,search_height*j+search_height),Scalar(100),2);

                line(display_image,Point(search_width*i+search_width,search_height*j+search_height),
                    Point(search_width*i+search_width,search_height*j),Scalar(100),2);

                Point3i temp;
                temp.x = matchLoc.x + target_image.cols/2 - search_width*i - search_width/2;
                temp.y = matchLoc.y + target_image.rows/2 - search_height*j - search_height/2;
                temp.z = 0;
                point_vec.push_back(temp);
            }
        }

        t_now = msg->header.stamp;
        dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec));
        t_last = t_now;

        /*if(have_pixhawk == true)
            flow_msg.header.stamp = header.stamp;
        else*/
            flow_msg.header.stamp = t_now;

        Point3i flow_result;
        if(filter_method == 0)
            flow_result = weng_method(point_vec);
        else
            flow_result = like_ransac(point_vec);
        point_vec.clear();
        int where_cols = (flow_result.z/((int)sqrt(patch_num)) + 0.5)*current_image.cols/sqrt(patch_num);
        int where_rows = (flow_result.z%((int)sqrt(patch_num)) + 0.5)*current_image.rows/sqrt(patch_num);
        circle(display_image,Point2i(where_cols,where_rows),6,Scalar(0),-1);

        flow_msg.header.frame_id = "template_match_flow";
        flow_msg.ground_distance = height;
        flow_msg.flow_x = flow_result.x/dt;
        flow_msg.flow_y = flow_result.y/dt;
        flow_msg.velocity_x = -1.*(flow_msg.flow_x/focus*height);
        flow_msg.velocity_y = flow_msg.flow_y/focus*height;
        flow_msg.quality = 0;
        flow_pub.publish(flow_msg);

        imshow("template_flow",display_image);
        createTrackbar(trackbar_label.c_str(),"template_flow",&match_method,max_Trackbar,NULL);
        createTrackbar("focus:","template_flow",&focus,max_f,NULL);
        createTrackbar("filter_method,0weng,1ransac:","template_flow",&filter_method,1,NULL);
        createTrackbar("save_img:","template_flow",&save_img,1,NULL);
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
