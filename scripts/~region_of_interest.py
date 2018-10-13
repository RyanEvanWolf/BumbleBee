#!/usr/bin/env python
import rospy
from bumblebee_calibration import *

import pickle

from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
from sensor_msgs.msg import CameraInfo,RegionOfInterest,Image
import tf.transformations
########Bumblebee

from cv_bridge import CvBridge


import copy

import argparse


parser = argparse.ArgumentParser(description='Given a topic and calibration settings, subsribes to the image topics and draws the Region of interest information')
 
# Required positional argument
parser.add_argument('calibration_directory', type=str,
                    help='The directory at which the pickled calibration information can be found for a stereo camera')

# Required positional argument
parser.add_argument('left_image_topic', type=str,
                    help='The topic under which the left image is being published')

# Optional positional argument
parser.add_argument('right_image_topic', type=str, nargs='?',
                    help='The topic under which the right image is being published')


args=parser.parse_args()

if(args.calibration_directory==None):
    parser.error("A calibration Directory is required to get ROI information")  
if(args.left_image_topic==None):
    parser.error("Left image topic has not been stipulated")

rospy.init_node('region_of_interest')

cvb=CvBridge()
intrin = pickle.load(open(args.calibration_directory + "/intrinsicCalibration.p", "rb"))
print("Camera Intrinsics Loaded")
overlapROI=getROIoverlap(intrin.lROI,intrin.rROI)

def pubROI(message,args):
    imageROI=copy.deepcopy(cvb.imgmsg_to_cv2(message))
    imageOverlap=copy.deepcopy(imageROI)

    cv2.rectangle(imageROI,
                    (args[0][1],args[0][0]),
                    (args[0][1]+args[0][3],args[0][0]+args[0][2]),
                    (0,0,255),
                    10)
    cv2.rectangle(imageROI,
                    (args[1][1],args[1][0]),
                    (args[1][1]+args[1][3],args[1][0]+args[1][2]),
                    (255,0,0),
                    10)
    cv2.rectangle(imageOverlap, (args[2][1],args[2][0]),
                   (args[2][1]+args[2][3],args[2][0]+args[2][2]),
                   (0,255,0),
                   -1)
    overallImage=cv2.addWeighted(imageROI, 0.7, imageOverlap, 0.2, 0)
    args[3].publish(cvb.cv2_to_imgmsg(overallImage))

#ROIimage=copy.deepcopy(drawROI(intrin.lROI,intrin.rROI,overlapROI))

pubLeft=rospy.Publisher(rospy.get_name()+args.left_image_topic,Image,queue_size=3,latch=True)
rospy.Subscriber(args.left_image_topic,Image,pubROI,(intrin.lROI,intrin.rROI,overlapROI,pubLeft))

if(args.right_image_topic!=None):
    pubRight=rospy.Publisher(rospy.get_name()+args.right_image_topic,Image,queue_size=3,latch=True)
    rospy.Subscriber(args.right_image_topic,Image,pubROI,(intrin.lROI,intrin.rROI,overlapROI,pubRight))





rospy.spin()


