#include <bumblebee/visoConfigurationManager.hpp>

visoConfigurationManager::visoConfigurationManager(std::string stereoInputFile)
{
  std::cout<<"Reading from "<<stereoInputFile<<std::endl;
	cv::FileStorage in(stereoInputFile,cv::FileStorage::READ);
	in["StereoRect"]>>bumblebee;
	in.release();
	ROS_INFO_STREAM("stereo File Loaded from :"<<stereoInputFile);
	///setup services
	ROS_INFO_STREAM(ros::this_node::getName()+"/getRectificationMap");
	getRectificationMapServ=n.advertiseService(ros::this_node::getName()+"/getRectificationMap",&visoConfigurationManager::RectificationMap,this);
	getOffsetServ=n.advertiseService(ros::this_node::getName()+"/getOffset",&visoConfigurationManager::Offset,this);
	getQServ=n.advertiseService(ros::this_node::getName()+"/getQ",&visoConfigurationManager::Q,this);
  getCameraInfoServ=n.advertiseService(ros::this_node::getName()+"/getCameraInfo",&visoConfigurationManager::CameraInfo,this);

}



bool visoConfigurationManager::Q(bumblebee::getQ::Request& req, bumblebee::getQ::Response& res)
{
	for(int rows=0;rows<4;rows++)
	{
		for(int cols=0;cols<4;cols++)
		{
			res.Q[rows*4+cols]=bumblebee.Qmap_.at<double>(rows,cols);
		}
	}
	return true;
}


bool visoConfigurationManager::CameraInfo(bumblebee::getCameraInfo::Request &req,bumblebee::getCameraInfo::Response &res)
{
  
  sensor_msgs::CameraInfo configMessageL, configMessageR;
	configMessageL.header.frame_id="/bumblebee_node/left/image_rect";
	configMessageR.header.frame_id="/bumblebee_node/right/image_rect";
	
	configMessageL.height=bumblebee.L_iMapx_.rows;
	configMessageL.width=bumblebee.L_iMapx_.cols;
	configMessageR.height=bumblebee.L_iMapx_.rows;
	configMessageR.width=bumblebee.L_iMapx_.cols;
	
	configMessageL.distortion_model="plumb_bob";
	configMessageR.distortion_model="plumb_bob";
	for(int index=0;index<5;index++)
	{
		configMessageL.D.push_back(0);
		configMessageR.D.push_back(0);
	}
	for(int row=0;row<3;row++)
	{
		configMessageL.R[3*row+row]=1;//set identity rotation
		configMessageR.R[3*row+row]=1;
		for(int col=0;col<3;col++)
		{
			configMessageL.K[3*row+col]=bumblebee.P_l_.at<double>(row,col);
			configMessageR.K[3*row+col]=bumblebee.P_r_.at<double>(row,col);
		}
	}
	
	for(int index=0;index<3;index++)
	{
		for(int col=0;col<4;col++)
		{
			configMessageL.P[4*index+col]=bumblebee.P_l_.at<double>(index,col);
			configMessageR.P[4*index+col]=bumblebee.P_r_.at<double>(index,col);
		}
	}
	
	configMessageL.roi.x_offset=bumblebee.l_ROI_.x;
	configMessageL.roi.y_offset=bumblebee.l_ROI_.y;
	configMessageL.roi.height=bumblebee.l_ROI_.height;
	configMessageL.roi.width=bumblebee.l_ROI_.width;
	
	configMessageR.roi.x_offset=bumblebee.r_ROI_.x;
	configMessageR.roi.y_offset=bumblebee.r_ROI_.y;
	configMessageR.roi.height=bumblebee.r_ROI_.height;
	configMessageR.roi.width=bumblebee.r_ROI_.width;
  
  res.left=configMessageL;
  res.right=configMessageR;
  return true;
}



bool visoConfigurationManager::RectificationMap(bumblebee::getRectificationMap::Request& req, bumblebee::getRectificationMap::Response& res)
{
	if(req.floatingPoint)
	{
		cv_bridge::CvImage(std_msgs::Header(),"32FC1",bumblebee.L_fMapx_).toImageMsg(res.leftx);
		cv_bridge::CvImage(std_msgs::Header(),"32FC1",bumblebee.L_fMapy_).toImageMsg(res.lefty);
		cv_bridge::CvImage(std_msgs::Header(),"32FC1",bumblebee.R_fMapx_).toImageMsg(res.rightx);
		cv_bridge::CvImage(std_msgs::Header(),"32FC1",bumblebee.R_fMapy_).toImageMsg(res.righty);
	}
	else
	{
		cv_bridge::CvImage(std_msgs::Header(),"16SC2",bumblebee.L_iMapx_).toImageMsg(res.leftx);
		cv_bridge::CvImage(std_msgs::Header(),"16SC2",bumblebee.L_iMapy_).toImageMsg(res.lefty);
		cv_bridge::CvImage(std_msgs::Header(),"16SC2",bumblebee.R_iMapx_).toImageMsg(res.rightx);
		cv_bridge::CvImage(std_msgs::Header(),"16SC2",bumblebee.R_iMapy_).toImageMsg(res.righty);
	}
	
	return true;
}



bool visoConfigurationManager::Offset(bumblebee::getOffset::Request& req, bumblebee::getOffset::Response& res)
{
	res.left.x_offset=bumblebee.l_ROI_.x;
	res.left.y_offset=bumblebee.l_ROI_.y;
	res.left.height=bumblebee.l_ROI_.height;
	res.left.width=bumblebee.l_ROI_.width;

	res.right.x_offset=bumblebee.r_ROI_.x;
	res.right.y_offset=bumblebee.r_ROI_.y;
	res.right.height=bumblebee.r_ROI_.height;
	res.right.width=bumblebee.r_ROI_.width;
	return true;
}
