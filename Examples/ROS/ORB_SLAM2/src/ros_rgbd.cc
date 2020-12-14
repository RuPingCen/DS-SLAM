/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
	ros::Publisher  pub_rgb,pub_depth,pub_tcw,pub_camerapath,pub_odom;
    size_t mcounter=0;	 
	nav_msgs::Path  camerapath;

    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
		pub_rgb= nh.advertise<sensor_msgs::Image> ("RGBImage", 10); 
		pub_depth= nh.advertise<sensor_msgs::Image> ("DepthImage", 10); 
		pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
		pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
		pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 
    }
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool  isKeyFrame =false;
	cv::Mat Tcw;

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),isKeyFrame);

    if (!Tcw.empty())
	{
        cv::Mat Twc =Tcw.inv();
        cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);  
        cv::Mat tWC=  Twc.rowRange(0,3).col(3);
  
        Eigen::Matrix<double,3,3> eigMat ;
        eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                        RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                        RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
        Eigen::Quaterniond q(eigMat);

        geometry_msgs::PoseStamped tcw_msg; 					
        tcw_msg.pose.position.x=tWC.at<float>(0);
        tcw_msg.pose.position.y=tWC.at<float>(1);			 
        tcw_msg.pose.position.z=tWC.at<float>(2);

        tcw_msg.pose.orientation.x=q.x();
        tcw_msg.pose.orientation.y=q.y();
        tcw_msg.pose.orientation.z=q.z();
        tcw_msg.pose.orientation.w=q.w();

            
        std_msgs::Header header ;
        header.stamp =msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id="world";
        
        sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
        sensor_msgs::Image::ConstPtr depth_msg=msgD;
        
        //geometry_msgs::PoseStamped tcw_msg; 
        tcw_msg.header=header;
        //tf::poseTFToMsg(tf_pose, tcw_msg.pose);
         
        // odometry information
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x=tWC.at<float>(0);
        odom_msg.pose.pose.position.y=tWC.at<float>(1);			 
        odom_msg.pose.pose.position.z=tWC.at<float>(2);
        
        odom_msg.pose.pose.orientation.x=q.x();
        odom_msg.pose.pose.orientation.y=q.y();
        odom_msg.pose.pose.orientation.z=q.z();
        odom_msg.pose.pose.orientation.w=q.w();
        
        odom_msg.header=header;
        odom_msg.child_frame_id="base_link"; 

        camerapath.header =header;
        camerapath.poses.push_back(tcw_msg);     
        pub_odom.publish(odom_msg);	  
         
        pub_camerapath.publish(camerapath);  
        if( isKeyFrame)
        {
            //pose information
            pub_tcw.publish(tcw_msg);	           
            pub_rgb.publish(rgb_msg);
            pub_depth.publish(depth_msg);	
        }
	}
	else
	{
	  cout<<"Twc is empty ..."<<endl;
	}
}


