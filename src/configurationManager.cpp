#include <bumblebee/configurationManager.hpp>

configurationManager::configurationManager(std::string stereoInputFile)
{

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
	//still to populate camera info properly TODO
	sensor_msgs::CameraInfo configMessageL, configMessageR;
	configMessageL.height=bumblebee.L_iMapx_.rows;
	configMessageL.width=bumblebee.L_iMapx_.cols;
	
	
	/*configMessageL.distortion_model="plumb_bob";
	configMessageL.D[0]=0;
	configMessageL.D[1]=0;
	configMessageL.D[2]=0;
	configMessageL.D[3]=0;
	configMessageL.D[4]=0;*/

	
	
	infoLPub.publish(configMessageL);
	infoRPub.publish(configMessageR);
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
