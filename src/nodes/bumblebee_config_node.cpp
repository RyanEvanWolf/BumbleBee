 
#include <ros/ros.h>

#include <Structures/CameraInfo/StereoRect.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/RegionOfInterest.h>


#include <bumblebee/configurationManager.hpp>

#define DEFAULT_RECTIFICATION_FILE "/home/ryan/git/Output/Calibration/stereo_ParameterFive.xml"

int main(int argc,char *argv[])
{
	std::string nodeName="bumblebee_configuration";
	ros::init(argc,argv,"bumblebee_configuration");
	configurationManager bumbleConfig(DEFAULT_RECTIFICATION_FILE);
	ros::spin();
  	return 0;
}
