#include <ros/ros.h>

#include <Structures/CameraInfo/StereoRect.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <bumblebee/getOffset.h>
#include <bumblebee/getRectificationMap.h>

cv::Rect lroi,rroi;
cv::Mat lmapx,lmapy;
cv::Mat rmapx,rmapy;

;
cv::Mat mainImage;
cv::Mat leftImage,rightImage;
cv::Mat leftROI,rightROI;

sensor_msgs::ImagePtr messageLroi,messageRroi;
sensor_msgs::ImagePtr messagecolour,messageL,messageR;


#define DEFAULT_RECTIFICATION_FILE "/home/ubuntu/ConfigurationFiles/stereo_ParameterFive.xml"
image_transport::Publisher *pubL,*pubR,*pubLroi,*pubRroi,*pubCol;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
		cv::cvtColor(cv_bridge::toCvShare(msg, "8UC1")->image,mainImage,cv::COLOR_BayerBG2RGB);

		cv::Mat grey;
		cv::cvtColor(mainImage,grey,CV_RGB2GRAY);
		
		rightImage=grey(cv::Rect(0,0,1024,768));
		cv::remap(rightImage,rightImage,rmapx,rmapy,cv::INTER_LINEAR);
		rightROI=rightImage(rroi);

		leftImage=grey(cv::Rect(0,768,1024,768));
		cv::remap(leftImage,leftImage,lmapx,lmapy,cv::INTER_LINEAR);
		leftROI=leftImage(lroi);


		messageLroi=cv_bridge::CvImage(std_msgs::Header(),"8UC1",leftROI).toImageMsg();
		messageRroi=cv_bridge::CvImage(std_msgs::Header(),"8UC1",rightROI).toImageMsg();
		messageL=cv_bridge::CvImage(std_msgs::Header(),"8UC1",leftImage).toImageMsg();
		messageR=cv_bridge::CvImage(std_msgs::Header(),"8UC1",rightImage).toImageMsg();

		pubLroi->publish(messageLroi);
		pubRroi->publish(messageRroi);
		pubL->publish(messageL);
		pubR->publish(messageR);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc,char *argv[])
{
	ros::init(argc,argv,"bumblebee");
	ros::NodeHandle n;

 	image_transport::ImageTransport it(n);
 	image_transport::Subscriber sub = it.subscribe("dataset/currentImage", 5, imageCallback);
	image_transport::Publisher  leftRectifiedPub=it.advertise("bumblebee/leftRectified",5);
	image_transport::Publisher 	rightRectifiedPub=it.advertise("bumblebee/rightRectified",5);
	image_transport::Publisher  leftROIPub=it.advertise("bumblebee/leftROI",5);
	image_transport::Publisher 	rightROIPub=it.advertise("bumblebee/rightROI",5);
	image_transport::Publisher 	colourpub=it.advertise("bumblebee/colour",5);
	
	
	
	//get bumblebee camera settings from services
	ros::ServiceClient mapClient = n.serviceClient<bumblebee::getRectificationMap>("bumblebee_configuration/getRectificationMap");
	ros::ServiceClient offsetClient=n.serviceClient<bumblebee::getOffset>("bumblebee_configuration/getOffset");

	//cv::FileStorage in(DEFAULT_RECTIFICATION_FILE,cv::FileStorage::READ);
	//in["StereoRect"]>>bumbleCamera;
	//in.release();

	bumblebee::getRectificationMap srv;
	srv.request.floatingPoint=true;
	if(mapClient.call(srv))
	{
		//ROS_INFO_STREAM(srv.response);
		cv_bridge::CvImagePtr temp;
		temp=cv_bridge::toCvCopy(srv.response.leftx,"32FC1");
		temp->image.copyTo(lmapx);
		temp=cv_bridge::toCvCopy(srv.response.lefty,"32FC1");
		temp->image.copyTo(lmapy);
		temp=cv_bridge::toCvCopy(srv.response.rightx,"32FC1");
		temp->image.copyTo(rmapx);
		temp=cv_bridge::toCvCopy(srv.response.righty,"32FC1");
		temp->image.copyTo(rmapy);
		
	}
	else
	{
		ROS_ERROR("getRectificationMap service was not found");
		return -1;
	}
	
	bumblebee::getOffset srv2;
	if(offsetClient.call(srv2))
	{
		lroi=cv::Rect(srv2.response.left.x_offset,
						srv2.response.left.y_offset,
						srv2.response.left.width,
						srv2.response.left.height);
		
		rroi=cv::Rect(srv2.response.right.x_offset,
						srv2.response.right.y_offset,
						srv2.response.right.width,
						srv2.response.right.height);
	}
	else
	{
		ROS_ERROR("getRectificationMap service was not found");
		return -1;
	}


	pubL = &leftRectifiedPub;
	pubR = &rightRectifiedPub;
	pubLroi = &leftROIPub;
	pubRroi= &rightROIPub;
	pubCol = &colourpub;


	ros::spin();
	
	return 0;
}
