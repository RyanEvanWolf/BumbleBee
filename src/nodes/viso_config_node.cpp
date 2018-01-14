 
#include <ros/ros.h>

#include <Structures/CameraInfo/StereoRect.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/RegionOfInterest.h>


#include <bumblebee/visoConfigurationManager.hpp>

#define DEFAULT_RECTIFICATION_FILE "/home/ryan/git/Output/Calibration/stereo_ParameterFive.xml"

int main(int argc,char *argv[])
{
	ros::init(argc,argv,"bumblebee_configuration");
  std::cout<<"Loading bumblebee_configuration from "<<argv[1]<<std::endl;
  std::string in(argv[1]);

	visoConfigurationManager bumbleConfig(in);//DEFAULT_RECTIFICATION_FILE);
	ros::spin();
  return 0;
}
 
