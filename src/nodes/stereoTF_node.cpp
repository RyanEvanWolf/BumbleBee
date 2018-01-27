#include <ros/console.h>
#include <ros/ros.h>  

#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Quaternion.h>
#include <Structures/CameraInfo/StereoRect.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core.hpp>

#include <iostream>
#include <math.h>

#define DEFAULT_RECTIFICATION_FILE "/home/ubuntu/ConfigurationFiles/stereo_ParameterFive.xml"
int main(int argc,char *argv[])
{
	ros::init(argc,argv,"bumblebee_stereo_tf_node");
	ros::NodeHandle n;

  std::string stereoInputFile(argv[1]);
  std::string baseName(argv[2]);
  std::string leftCameraName(argv[3]);
  std::string rightCameraName(argv[4]);

  stereo::StereoRect bumblebee;
  cv::FileStorage in(stereoInputFile,cv::FileStorage::READ);
	in["StereoRect"]>>bumblebee;
	in.release();
	ROS_INFO_STREAM("stereo TF File Loaded from :"<<stereoInputFile);
  
  geometry_msgs::TransformStamped lRectifiedmsg,rRectifiedmsg,baselinkmsg;
  tf2::Quaternion quat;
  //create left rectified camera tf
  //set as origin 
  lRectifiedmsg.header.frame_id="baseLink";
  lRectifiedmsg.child_frame_id="/bumblebee/left";
  lRectifiedmsg.transform.translation.x = 0.0;
  lRectifiedmsg.transform.translation.y = 0.0;
  lRectifiedmsg.transform.translation.z = 0.0;
  
  quat.setRPY(0.0,0.0,0.0);// M_PI/2.0, 0.0);
  lRectifiedmsg.transform.rotation.x = quat.x();
  lRectifiedmsg.transform.rotation.y = quat.y();
  lRectifiedmsg.transform.rotation.z = quat.z();
  lRectifiedmsg.transform.rotation.w = quat.w();
  //create right rectified camera tf

  rRectifiedmsg.header.frame_id="/bumblebee/left";
  rRectifiedmsg.child_frame_id="/bumblebee/right";
  //calculate Ideal baseline
  double baselineX=bumblebee.P_r_.at<double>(0,3)/bumblebee.P_r_.at<double>(0,0);
  rRectifiedmsg.transform.translation.x=baselineX;///1000.0;
  rRectifiedmsg.transform.translation.y=0.0;
  rRectifiedmsg.transform.translation.z=0.0;
  quat.setRPY(0.0,0.0,0.0);
  rRectifiedmsg.transform.rotation.x=quat.x();
  rRectifiedmsg.transform.rotation.y=quat.y();
  rRectifiedmsg.transform.rotation.z=quat.z();
  rRectifiedmsg.transform.rotation.w=quat.w();

  
  tf2_ros::TransformBroadcaster mainPub;
  
  ros::Rate loop_rate(15);
  
  
  while(ros::ok())
  {
    lRectifiedmsg.header.stamp = ros::Time::now();
    rRectifiedmsg.header.stamp = ros::Time::now();
    mainPub.sendTransform(lRectifiedmsg);
    mainPub.sendTransform(rRectifiedmsg);
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  return 0;
}