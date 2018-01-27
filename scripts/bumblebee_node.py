#!/usr/bin/env python
#####ros related imports
import rospy
from cv_bridge import CvBridge
##bumblebee services
from bumblebee.srv import getRectificationMap,getRectificationMapRequest,getRectificationMapResponse
from bumblebee.srv import getOffset,getOffsetRequest,getOffsetResponse
from bumblebee.srv import getCameraInfo,getCameraInfoRequest,getCameraInfoResponse
#message types
from std_msgs.msg import Bool
from sensor_msgs.msg import Image,CameraInfo
import copy

import time

import cv2


rospy.init_node('bumblebee')


##subscribers and publish topics/services

leftPublishers = []
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/image_raw",Image,queue_size=8))
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/image",Image,queue_size=8))
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/image_color",Image,queue_size=8))
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/image_rect",Image,queue_size=8))
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/image_rect_color",Image,queue_size=8))
leftPublishers.append(rospy.Publisher(rospy.get_name()+"/left/ROI",Image,queue_size=8))

rightPublishers = []
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/image_raw",Image,queue_size=8))
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/image",Image,queue_size=8))
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/image_color",Image,queue_size=8))
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/image_rect",Image,queue_size=8))
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/image_rect_color",Image,queue_size=8))
rightPublishers.append(rospy.Publisher(rospy.get_name()+"/right/ROI",Image,queue_size=8))

imgTopic="/dataset/currentImage"
rectMapTopic="/bumblebee_configuration/getRectificationMap"
offsetTopic="/bumblebee_configuration/getOffset"
infoTopic="/bumblebee_configuration/getCameraInfo"


leftinfoPub=rospy.Publisher(rospy.get_name()+"/left/camera_info",CameraInfo,queue_size=8)
rightinfoPub=rospy.Publisher(rospy.get_name()+"/right/camera_info",CameraInfo,queue_size=8)


time.sleep(10)## just wait for the other nodes to setup
print("bumblebee stereo node started")
cvb = CvBridge()
rospy.wait_for_service(rectMapTopic, 10)
rectMapServ= rospy.ServiceProxy(rectMapTopic, getRectificationMap)
offsetServ=rospy.ServiceProxy(offsetTopic, getOffset)
infoServ=rospy.ServiceProxy(infoTopic,getCameraInfo)

rectMessage= getRectificationMapRequest()
rectMessage.floatingPoint=True

reply=rectMapServ(rectMessage)

offMessage=getOffsetRequest()
offreply=offsetServ(offMessage)

global lxMap,lyMap,rxMap,ryMap
global lOffset,rOffset
global lInfo,rInfo

lxMap=copy.deepcopy(cvb.imgmsg_to_cv2(reply.leftx))
lyMap=copy.deepcopy(cvb.imgmsg_to_cv2(reply.lefty))
rxMap=copy.deepcopy(cvb.imgmsg_to_cv2(reply.rightx))
ryMap=copy.deepcopy(cvb.imgmsg_to_cv2(reply.righty))

info_reply=infoServ(getCameraInfoRequest())

lInfo=copy.deepcopy(info_reply.left)
rInfo=copy.deepcopy(info_reply.right)

def pubTopic(message):
    fullImage = cvb.imgmsg_to_cv2(message)
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

    limages.append(copy.deepcopy(cv2.remap(limages[1],lxMap,lyMap,cv2.INTER_LINEAR)))
    rimages.append(copy.deepcopy(cv2.remap(rimages[1], rxMap, ryMap, cv2.INTER_LINEAR)))
    imageEncoding.append("mono8")

    limages.append(copy.deepcopy(cv2.remap(limages[2],lxMap,lyMap,cv2.INTER_LINEAR)))
    rimages.append(copy.deepcopy(cv2.remap(rimages[2], rxMap, ryMap, cv2.INTER_LINEAR)))
    imageEncoding.append("rgb8")

    print(offreply)
    print(limages[3][offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width].shape)
    limages.append(copy.deepcopy(limages[3][offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width]))
    rimages.append(copy.deepcopy(rimages[3][offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width]))
    imageEncoding.append("mono8")
    #cv2.cvtColor(cv2.imread(i, cv2.IMREAD_GRAYSCALE), cv2.COLOR_BAYER_BG2RGB))11


    # colourL = cv2.remap(colourL,lxMap,lyMap,cv2.INTER_LINEAR)
    # colourR=cv2.remap(colourR,rxMap,ryMap,cv2.INTER_LINEAR)
    # publrect.publish(cvb.cv2_to_imgmsg(colourL))
    # pubrrect.publish(cvb.cv2_to_imgmsg(colourR))
    # colourL=cv2.cvtColor(colourL[offreply.left.y_offset:offreply.left.height,offreply.left.x_offset:offreply.left.width],cv2.COLOR_RGB2GRAY)
    # #colourL=cv2.equalizeHist(colourL)
    # leftGrey=cvb.cv2_to_imgmsg(colourL)
    # leftGrey.header.frame_id="RectifiedLeftFrame"
    # leftGrey.header.stamp=rospy.get_rostime()
    #
    # publ.publish(leftGrey)
    # colourR = cv2.cvtColor(colourR[offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width],cv2.COLOR_RGB2GRAY)
    # #colourR=cv2.equalizeHist(colourR)
    #
    # rightGrey=cvb.cv2_to_imgmsg(colourR)
    # rightGrey.header.frame_id="RectifiedRightFrame"
    # rightGrey.header.stamp=rospy.get_rostime()
    # pubr.publish(rightGrey)
    pubTime=rospy.get_rostime()
    lInfo.header.stamp=pubTime
    rInfo.header.stamp=pubTime

    leftinfoPub.publish(lInfo)
    rightinfoPub.publish(rInfo)

    ##
    for index in range(0,len(limages)):
        lImageMessage=cvb.cv2_to_imgmsg(limages[index])
        lImageMessage.header.stamp=pubTime
        lImageMessage.header.frame_id="/left"
        lImageMessage.encoding=imageEncoding[index]
        rImageMessage=cvb.cv2_to_imgmsg(rimages[index])
        rImageMessage.header.stamp=pubTime
        rImageMessage.header.frame_id="/right"
        rImageMessage.encoding = imageEncoding[index]
        leftPublishers[index].publish(lImageMessage)
        rightPublishers[index].publish(rImageMessage)




rospy.Subscriber(imgTopic, Image, pubTopic)



rospy.spin()