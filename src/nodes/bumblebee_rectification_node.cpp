#include <ros/ros.h>

#include <Structures/CameraInfo/StereoRect.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <bumblebee/imageRectification.h>

cv::Mat lmapIx,lmapIy;
cv::Mat lmapFx,lmapFy;


cv::Mat rmapIx,rmapIy;
cv::Mat rmapFx,rmapFy;
int serviceRequestCount=0;

//ros::ServiceServer service = n.advertiseService("add_two_array", add);

void lIx(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	lmapIx=(cv_bridge::toCvShare(msg, "16SC2")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
	std::cout<<lmapIx.size()<<std::endl;
}

void lIy(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	lmapIy=(cv_bridge::toCvShare(msg, "16UC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
	std::cout<<lmapIy.size()<<std::endl;
}

void rIx(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	rmapIx=(cv_bridge::toCvShare(msg, "16SC2")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}

void rIy(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	rmapIy=(cv_bridge::toCvShare(msg, "16UC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}


void lFx(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	lmapFx=(cv_bridge::toCvShare(msg,"32FC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}

void lFy(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	lmapFy=(cv_bridge::toCvShare(msg,"32FC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}

void rFx(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	rmapFx=(cv_bridge::toCvShare(msg,"32FC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}

void rFy(const sensor_msgs::ImageConstPtr& msg)
{
	std::cout<<msg->encoding<<std::endl;
	rmapFy=(cv_bridge::toCvShare(msg,"32FC1")->image).clone();//,lmapIx);//,cv::COLOR_BayerBG2RGB);
}

bool srvRectification(bumblebee::imageRectification::Request & req,
						bumblebee::imageRectification::Response & res)
{
	cv::Mat inputImage;
	cv::Mat debayerL,debayerR;
	cv::Mat rectL,rectR;
	cv::Mat lImage,rImage;
	struct timeval  tv1, tv2;
	res.debayerTime=0;
	res.rectificationTime=0;
	res.intRectificationTime=0;
	float executionTime;

	inputImage=(cv_bridge::toCvCopy(req.fullImage,"8UC1")->image);
	lImage=inputImage(cv::Rect(0,768,1024,768)).clone();
	rImage=inputImage(cv::Rect(0,0,1024,768)).clone();
	debayerL=lImage.clone();
	debayerR=rImage.clone();

	gettimeofday(&tv1, NULL);
	cv::cvtColor(lImage,debayerL,cv::COLOR_BayerBG2GRAY);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.debayerTime+=executionTime;

	gettimeofday(&tv1, NULL);
	cv::cvtColor(rImage,debayerR,cv::COLOR_BayerBG2GRAY);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.debayerTime+=executionTime;		

	rectL=debayerL.clone();

	gettimeofday(&tv1, NULL);
	cv::remap(debayerL,rectL,lmapFx,lmapFy,cv::INTER_LINEAR);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.rectificationTime+=executionTime;

	gettimeofday(&tv1, NULL);
	cv::remap(debayerR,rectR,rmapFx,rmapFy,cv::INTER_LINEAR);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.rectificationTime+=executionTime;

	gettimeofday(&tv1, NULL);
	cv::remap(debayerL,rectL,lmapIx,lmapIy,cv::INTER_LINEAR);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.intRectificationTime+=executionTime;

	gettimeofday(&tv1, NULL);
	cv::remap(debayerR,rectR,rmapIx,rmapIy,cv::INTER_LINEAR);
	gettimeofday(&tv2, NULL);
	executionTime=((float)(tv2.tv_usec - tv1.tv_usec) / 1000000)+((float)(tv2.tv_sec - tv1.tv_sec));
	res.intRectificationTime+=executionTime;
	//cv::remap(debayerR,rectR,rmapIx,rmapIy,cv::INTER_LINEAR);


// cv::Mat lmapIx,lmapIy;
// cv::Mat lmapFx,lmapFy;


// cv::Mat rmapIx,rmapIy;
// cv::Mat rmapFx,rmapFy;
	serviceRequestCount++;
	std::cout<<serviceRequestCount<<std::endl;

	res.lRectified=*(cv_bridge::CvImage(std_msgs::Header(),"8UC1",rectL).toImageMsg());
	res.rRectified=*(cv_bridge::CvImage(std_msgs::Header(),"8UC1",rectR).toImageMsg());
	return true;
}
/*
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
*/
int main(int argc,char *argv[])
{
	ros::init(argc,argv,"bumblebee_rectification_node");
	ros::NodeHandle n;
	std::cout<<"Waiting for topics"<<std::endl;

	image_transport::ImageTransport it(n);

	ros::ServiceServer service = n.advertiseService("bumblebee_rectification_node/singleImageRectification",srvRectification );

 	image_transport::Subscriber sub = it.subscribe("/bumblebee_configuration/idealLeft/intX", 5,lIx);
	image_transport::Subscriber sub1 = it.subscribe("/bumblebee_configuration/idealLeft/intY", 5,lIy);
	image_transport::Subscriber sub2 = it.subscribe("/bumblebee_configuration/idealRight/intX", 5,rIx);
	image_transport::Subscriber sub3 = it.subscribe("/bumblebee_configuration/idealRight/intY", 5,rIy);


 	image_transport::Subscriber sub4 = it.subscribe("/bumblebee_configuration/idealLeft/floatX", 5,lFx);
	image_transport::Subscriber sub5 = it.subscribe("/bumblebee_configuration/idealLeft/floatY", 5,lFy);
	image_transport::Subscriber sub6 = it.subscribe("/bumblebee_configuration/idealRight/floatX", 5,rFx);
	image_transport::Subscriber sub7 = it.subscribe("/bumblebee_configuration/idealRight/floatY", 5,rFy);


	std::cout<<"All settings loaded,spinning"<<std::endl;
	ros::spin();
	
	return 0;
} 

