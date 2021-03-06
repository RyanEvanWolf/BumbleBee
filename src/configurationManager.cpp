#include <bumblebee/configurationManager.hpp>

configurationManager::configurationManager(std::string stereoInputFile)
{
  std::cout<<"beginning Read\n";
  
	cv::FileStorage in(stereoInputFile,cv::FileStorage::READ);
	in["StereoRect"]>>bumblebee;
	in.release();
	ROS_INFO_STREAM("stereo File Loaded from :"<<stereoInputFile);
	///setup services
	
	getRectificationMapServ=n.advertiseService("bumblebee_configuration/getRectificationMap",&configurationManager::RectificationMap,this);
	getOffsetServ=n.advertiseService("bumblebee_configuration/getOffset",&configurationManager::Offset,this);
	getQServ=n.advertiseService("bumblebee_configuration/getQ",&configurationManager::Q,this);
	infoLPub=n.advertise<sensor_msgs::CameraInfo>("bumblebee_configuration/left/info",20,true);
	infoRPub=n.advertise<sensor_msgs::CameraInfo>("bumblebee_configuration/right/info",20,true);
	
	//assuming the cameraInfo is ideal rectified data
	//i.e. no rotation information really required, everything set to identity
	sensor_msgs::CameraInfo configMessageL, configMessageR;
	configMessageL.header.frame_id="RectifiedLeftFrame";
	configMessageR.header.frame_id="RectifiedRightFrame";
	
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
	
	infoLPub.publish(configMessageL);
	infoRPub.publish(configMessageR);
  
  //set baselink to the camera sensor frame
}



bool configurationManager::Q(bumblebee::getQ::Request& req, bumblebee::getQ::Response& res)
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


bool configurationManager::RectificationMap(bumblebee::getRectificationMap::Request& req, bumblebee::getRectificationMap::Response& res)
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



bool configurationManager::Offset(bumblebee::getOffset::Request& req, bumblebee::getOffset::Response& res)
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
