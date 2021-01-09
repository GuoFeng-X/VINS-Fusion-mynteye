/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;


queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}


void sync_process()
{
    while(1)
    {
		cv::Mat image0, image1;
		std_msgs::Header header;
		double time = 0;
		m_buf.lock();
		if (!img0_buf.empty() && !img1_buf.empty())
		{
			double time0 = img0_buf.front()->header.stamp.toSec();
			double time1 = img1_buf.front()->header.stamp.toSec();
			// 0.003s sync tolerance
			if(time0 < time1 - 0.003)
			{
				img0_buf.pop();
				printf("throw img0\n");
			}
			else if(time0 > time1 + 0.003)
			{
				img1_buf.pop();
				printf("throw img1\n");
			}
			else
			{
				time = img0_buf.front()->header.stamp.toSec();
				header = img0_buf.front()->header;
				image0 = getImageFromMsg(img0_buf.front());
				img0_buf.pop();
				image1 = getImageFromMsg(img1_buf.front());
				img1_buf.pop();
			}
		}
		m_buf.unlock();
		if(!image0.empty())
		{
			estimator.inputImage(time, image0, image1);

			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			printf ("%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
															pose(1,0), pose(1,1), pose(1,2),pose(1,3),
															pose(2,0), pose(2,1), pose(2,2),pose(2,3));
		}
        
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	// if(argc != 1)
	// {
	// 	printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
	// 		   "for example: rosrun vins kitti_odom_test "
	// 		   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml \n");
	// 	return 1;
	// }


	// string config_file = argv[1];
	// printf("config_file: %s\n", argv[1]);

	string config_file;
	n.getParam("config_path", config_file);
	std::cout<<"config_file: "<<config_file<<std::endl;
	// printf("config_file: %s\n", config_file);


	readParameters(config_file);
	estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;

}
