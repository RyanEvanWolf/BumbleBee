#!/usr/bin/env python
#####ros related imports
import rospy
from cv_bridge import CvBridge
#message types
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,CameraInfo


import copy

import time
import sys
import cv2

import pickle
import numpy as np

lMap = pickle.load(open("/home/ryan/currentTesting/stereoOut/left_Map.p", "rb"))
#
#
#list of bumbleBee topics
# /bumblebee_configuration/Q
# /bumblebee_configuration/idealLeft/CameraInfo
# /bumblebee_configuration/idealLeft/floatX
# /bumblebee_configuration/idealLeft/floatY
# /bumblebee_configuration/idealLeft/intX
# /bumblebee_configuration/idealLeft/intY
# /bumblebee_configuration/idealRight/CameraInfo
# /bumblebee_configuration/idealRight/floatX
# /bumblebee_configuration/idealRight/floatY
# /bumblebee_configuration/idealRight/intX
# /bumblebee_configuration/idealRight/intY
#
#

class bumblebeeCamera():
    def __init__(self,ConfigurationName):
        self.cvb=CvBridge()
        print("Bumblebee Configuration Topic = " + ConfigurationName)
        self.imgTopic="/dataset/currentImage"##assumed raw bumblebee bayer image
        self.configTopicName=ConfigurationName
        self.Q=None
        self.Info=[None,None]
        self.Mapping=[[None,None],[None,None],[None,None],[None,None]]
        self.ROI=[None,None]
        self.Publishers=[]
        self.Subscribers=[]
        ##[Left Right]
        #######Create the Publish Topics
        leftPub=[]
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/image_raw",Image,queue_size=8))
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/image",Image,queue_size=8))
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/image_color",Image,queue_size=8))
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/image_rect",Image,queue_size=8))
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/image_rect_color",Image,queue_size=8))
        leftPub.append(rospy.Publisher(rospy.get_name()+"/left/ROI",Image,queue_size=8))
        self.Publishers.append(leftPub)
        rightPub=[]
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/image_raw",Image,queue_size=8))
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/image",Image,queue_size=8))
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/image_color",Image,queue_size=8))
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/image_rect",Image,queue_size=8))
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/image_rect_color",Image,queue_size=8))
        rightPub.append(rospy.Publisher(rospy.get_name()+"/right/ROI",Image,queue_size=8))
        self.Publishers.append(rightPub)
        #####Create Subscriptions to Topics
        self.Subscribers.append(rospy.Subscriber("/bumblebee_configuration/idealLeft/CameraInfo",CameraInfo,self.subInfo,(0)))
        self.Subscribers.append(rospy.Subscriber("/bumblebee_configuration/idealRight/CameraInfo", CameraInfo, self.subInfo, (1)))
        self.Subscribers.append(rospy.Subscriber("/bumblebee_configuration/idealLeft/intX",Image, self.subMapping, (0,0)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealLeft/intY", Image, self.subMapping, (0, 1)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealLeft/floatX", Image, self.subMapping, (1, 0)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealLeft/floatY", Image, self.subMapping, (1, 1)))

        self.Subscribers.append(rospy.Subscriber("/bumblebee_configuration/idealRight/intX",Image, self.subMapping, (2,0)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealRight/intY", Image, self.subMapping, (2, 1)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealRight/floatX", Image, self.subMapping, (3, 0)))
        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/idealRight/floatY", Image, self.subMapping, (3, 1)))

        self.Subscribers.append(
            rospy.Subscriber("/bumblebee_configuration/Q", Image, self.subQ))

        rospy.Subscriber(self.imgTopic, Image, self.pubTopic)
    def subInfo(self,message,args):
        print("updating Camera Info "+str(args))
        ##args being the index in Info which must be updated
        self.Info[args]=copy.deepcopy(message)
        self.ROI[args]=([message.roi.x_offset,
                         message.roi.y_offset,
                         message.roi.height,
                         message.roi.width])
        if((self.ROI[0]!=None) and (self.ROI[1]!=None)):
            pass
    def subMapping(self,message,args):
        print("updating Mapping ["+str(args[0])+"]["+str(args[1])+"]")
        self.Mapping[args[0]][args[1]]=copy.deepcopy(self.cvb.imgmsg_to_cv2(message))
    def subQ(self,message):
        print("updating Q")
        self.Q=copy.deepcopy(self.cvb.imgmsg_to_cv2(message))
    def subTopic(self,message,args):
        print("Updated Settings on topic "+args[1])
        args[0]=copy.deepcopy(message)
    def pubTopic(self,message):
        ###generate the images
        fullImage = self.cvb.imgmsg_to_cv2(message)
        limages=[]
        rimages=[]
        imageEncoding=[]

        limages.append(copy.deepcopy(fullImage[768:768*2,0:1024]))
        rimages.append(copy.deepcopy(fullImage[0:768, 0:1024]))
        imageEncoding.append("mono8")

        limages.append(copy.deepcopy(cv2.cvtColor(limages[0],cv2.COLOR_BAYER_BG2GRAY)))
        rimages.append(copy.deepcopy(cv2.cvtColor(rimages[0],cv2.COLOR_BAYER_BG2GRAY)))
        imageEncoding.append("mono8")


        limages.append(copy.deepcopy(cv2.cvtColor(limages[0],cv2.COLOR_BAYER_BG2RGB)))
        rimages.append(copy.deepcopy(cv2.cvtColor(rimages[0],cv2.COLOR_BAYER_BG2RGB)))
        imageEncoding.append("rgb8")


        limages.append(copy.deepcopy(cv2.remap(limages[1],self.Mapping[1][0],self.Mapping[1][1],cv2.INTER_LINEAR)))
        rimages.append(copy.deepcopy(cv2.remap(rimages[1],self.Mapping[3][0],self.Mapping[3][1], cv2.INTER_LINEAR)))
        imageEncoding.append("mono8")

        limages.append(copy.deepcopy(cv2.remap(limages[2],self.Mapping[1][0],self.Mapping[1][1],cv2.INTER_LINEAR)))
        rimages.append(copy.deepcopy(cv2.remap(rimages[2],self.Mapping[1][0],self.Mapping[1][1],cv2.INTER_LINEAR)))
        imageEncoding.append("rgb8")

        limages.append(copy.deepcopy(limages[3][self.ROI[0][1]:self.ROI[0][2],self.ROI[0][0]:self.ROI[0][3]]))
        rimages.append(copy.deepcopy(rimages[3][self.ROI[1][1]:self.ROI[1][2],self.ROI[1][0]:self.ROI[1][3]]))#offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width]))
        imageEncoding.append("mono8")
        pubTime = rospy.get_rostime()

        self.Info[0].header.stamp = pubTime
        self.Info[1].header.stamp=pubTime
        leftinfoPub.publish(lInfo)
        rightinfoPub.publish(rInfo)
        for index in range(0,len(limages)):
            lImageMessage=self.cvb.cv2_to_imgmsg(limages[index])
            lImageMessage.header.stamp=pubTime
            lImageMessage.header.frame_id="/left"
            lImageMessage.encoding=imageEncoding[index]
            rImageMessage=self.cvb.cv2_to_imgmsg(rimages[index])
            rImageMessage.header.stamp=pubTime
            rImageMessage.header.frame_id="/right"
            rImageMessage.encoding = imageEncoding[index]
            self.Publishers[0][index].publish(lImageMessage)
            self.Publishers[1][index].publish(rImageMessage)

rospy.init_node('bumblebee')

test=bumblebeeCamera(sys.argv[1])

#
# worldFrame="world"
# leftCameraFrame="left"
# rightCameraFrame="right"
# rightRectifiedFrame="rectified_right"
# leftRectifiedFrame="rectified_left"
#
# def pubTopic(message):
#     fullImage = cvb.imgmsg_to_cv2(message)
#     limages=[]
#     rimages=[]
#     imageEncoding=[]
#
#     limages.append(copy.deepcopy(fullImage[768:768*2,0:1024]))
#     rimages.append(copy.deepcopy(fullImage[0:768, 0:1024]))
#     imageEncoding.append("mono8")
#
#     limages.append(copy.deepcopy(cv2.cvtColor(limages[0],cv2.COLOR_BAYER_BG2GRAY)))
#     rimages.append(copy.deepcopy(cv2.cvtColor(rimages[0],cv2.COLOR_BAYER_BG2GRAY)))
#     imageEncoding.append("mono8")
#
#
#     limages.append(copy.deepcopy(cv2.cvtColor(limages[0],cv2.COLOR_BAYER_BG2RGB)))
#     rimages.append(copy.deepcopy(cv2.cvtColor(rimages[0],cv2.COLOR_BAYER_BG2RGB)))
#     imageEncoding.append("rgb8")
#
#     limages.append(copy.deepcopy(cv2.remap(limages[1],lxMap,lyMap,cv2.INTER_LINEAR)))
#     rimages.append(copy.deepcopy(cv2.remap(rimages[1], rxMap, ryMap, cv2.INTER_LINEAR)))
#     imageEncoding.append("mono8")
#
#     limages.append(copy.deepcopy(cv2.remap(limages[2],lxMap,lyMap,cv2.INTER_LINEAR)))
#     rimages.append(copy.deepcopy(cv2.remap(rimages[2], rxMap, ryMap, cv2.INTER_LINEAR)))
#     imageEncoding.append("rgb8")
#
#     print(offreply)
#     print(limages[3][offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width].shape)
#     limages.append(copy.deepcopy(limages[3][offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width]))
#     rimages.append(copy.deepcopy(rimages[3][offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width]))
#     imageEncoding.append("mono8")
                #     cv2.cvtColor(cv2.imread(i, cv2.IMREAD_GRAYSCALE), cv2.COLOR_BAYER_BG2RGB))11
                #
                #
                #     colourL = cv2.remap(colourL,lxMap,lyMap,cv2.INTER_LINEAR)
                #     colourR=cv2.remap(colourR,rxMap,ryMap,cv2.INTER_LINEAR)
                #     publrect.publish(cvb.cv2_to_imgmsg(colourL))
                #     pubrrect.publish(cvb.cv2_to_imgmsg(colourR))
                #     colourL=cv2.cvtColor(colourL[offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width],cv2.COLOR_RGB2GRAY)
                #     #colourL=cv2.equalizeHist(colourL)
                #     leftGrey=cvb.cv2_to_imgmsg(colourL)
                #     leftGrey.header.frame_id="RectifiedLeftFrame"
                #     leftGrey.header.stamp=rospy.get_rostime()
                #
                #     publ.publish(leftGrey)
                #     colourR = cv2.cvtColor(colourR[offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width],cv2.COLOR_RGB2GRAY)
                #     #colourR=cv2.equalizeHist(colourR)
                #
                #     rightGrey=cvb.cv2_to_imgmsg(colourR)
                #     rightGrey.header.frame_id="RectifiedRightFrame"
                #     rightGrey.header.stamp=rospy.get_rostime()
#     # pubr.publish(rightGrey)
#     pubTime=rospy.get_rostime()
#     lInfo.header.stamp=pubTime
#     rInfo.header.stamp=pubTime
#
#     leftinfoPub.publish(lInfo)
#     rightinfoPub.publish(rInfo)
#
#     ##
#     for index in range(0,len(limages)):
#         lImageMessage=cvb.cv2_to_imgmsg(limages[index])
#         lImageMessage.header.stamp=pubTime
#         lImageMessage.header.frame_id="/left"
#         lImageMessage.encoding=imageEncoding[index]
#         rImageMessage=cvb.cv2_to_imgmsg(rimages[index])
#         rImageMessage.header.stamp=pubTime
#         rImageMessage.header.frame_id="/right"
#         rImageMessage.encoding = imageEncoding[index]
#         leftPublishers[index].publish(lImageMessage)
#         rightPublishers[index].publish(rImageMessage)
#
#
#
#
# rospy.Subscriber(imgTopic, Image, pubTopic)
#
#
#
rospy.spin()