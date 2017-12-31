#ifndef BUMBLEBEE_CONFIGURATION_MANAGER_HPP
#define BUMBLEBEE_CONFIGURATION_MANAGER_HPP

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
#include <iostream>

class configurationManager
{
	private:
		ros::Publisher infoLPub,infoRPub;
		ros::ServiceServer getRectificationMapServ;
		ros::ServiceServer getOffsetServ;
		ros::ServiceServer getQServ;
		bool RectificationMap(bumblebee::getRectificationMap::Request &req,bumblebee::getRectificationMap::Response &res);
		bool Offset(bumblebee::getOffset::Request &req,bumblebee::getOffset::Response &res);
		bool Q(bumblebee::getQ::Request &req,bumblebee::getQ::Response &res);
	public:
		ros::NodeHandle n;
		stereo::StereoRect bumblebee;
		configurationManager(std::string stereoInputFile);
};


#endif
