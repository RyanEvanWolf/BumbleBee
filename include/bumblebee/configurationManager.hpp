#ifndef BUMBLEBEE_CONFIGURATION_MANAGER_HPP
#define BUMBLEBEE_CONFIGURATION_MANAGER_HPP

#include <ros/console.h>
#include <ros/ros.h> 

//ros common messages
#include <sensor_msgs/RegionOfInterest.h>



#include <Structures/CameraInfo/StereoRect.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
/////custom messages and services
///messages


//services
#include <bumblebee/getRectificationMap.h>
#include <bumblebee/getOffset.h>


class configurationManager
{
	private:
		ros::ServiceServer getRectificationMapServ;
		ros::ServiceServer getOffsetServ;
		bool RectificationMap(bumblebee::getRectificationMap::Request &req,bumblebee::getRectificationMap::Response &res);
		bool Offset(bumblebee::getOffset::Request &req,bumblebee::getOffset::Response &res);
	public:
		ros::NodeHandle n;
		stereo::StereoRect bumblebee;
		configurationManager(std::string stereoInputFile);
};


#endif
