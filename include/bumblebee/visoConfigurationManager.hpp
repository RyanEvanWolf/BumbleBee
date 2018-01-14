#ifndef VISO_CONFIGURATION_MANAGER_HPP
#define VISO_CONFIGURATION_MANAGER_HPP

#include <ros/console.h>
#include <ros/ros.h> 

//ros common messages
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>


#include <Structures/CameraInfo/StereoRect.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
/////custom messages and services
///messages
//services
#include <bumblebee/getRectificationMap.h>
#include <bumblebee/getOffset.h>
#include <bumblebee/getQ.h>
#include <bumblebee/getCameraInfo.h>

#include <iostream>

class visoConfigurationManager
{
	private:
    ros::ServiceServer getRectificationMapServ;
		ros::ServiceServer getOffsetServ;
		ros::ServiceServer getQServ;
    ros::ServiceServer getCameraInfoServ;
		bool RectificationMap(bumblebee::getRectificationMap::Request &req,bumblebee::getRectificationMap::Response &res);
		bool Offset(bumblebee::getOffset::Request &req,bumblebee::getOffset::Response &res);
		bool Q(bumblebee::getQ::Request &req,bumblebee::getQ::Response &res);
    bool CameraInfo(bumblebee::getCameraInfo::Request &req,bumblebee::getCameraInfo::Response &res);
	public:
		ros::NodeHandle n;
		stereo::StereoRect bumblebee;
		visoConfigurationManager(std::string stereoInputFile);
};


#endif 
 
