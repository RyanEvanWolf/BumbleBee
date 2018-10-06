 
import cv2
import math
import time
import random
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
import copy

def getCameraSettingsFromServer(serverNodeName="/bumblebee_configuration"):
    cvb=CvBridge()
    ##assumes a node has been declared
    cameraSettings={}
    cameraSettings["Q"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Q",Image))
    cameraSettings["lInfo"]=rospy.wait_for_message(serverNodeName+"/ideal/leftRectified/CameraInfo",CameraInfo)
    cameraSettings["rInfo"]=rospy.wait_for_message(serverNodeName+"/ideal/rightRectified/CameraInfo",CameraInfo)
    cameraSettings["Pl"]=np.zeros((3,4),dtype=np.float64)
    cameraSettings["Pr"]=np.zeros((3,4),dtype=np.float64)
    for row in range(0,3):
            for col in range(0,4):
                cameraSettings["Pl"][row,col]=cameraSettings["lInfo"].P[row*4 +col]
                cameraSettings["Pr"][row,col]=cameraSettings["rInfo"].P[row*4 +col]
    
    

    lmapixPub=rospy.Publisher(rospy.get_name()+"/idealLeft/intX",Image,queue_size=3,latch=True)
    lmapiyPub = rospy.Publisher(rospy.get_name() + "/idealLeft/intY", Image, queue_size=3, latch=True)
    lmapfxPub = rospy.Publisher(rospy.get_name() + "/idealLeft/floatX", Image, queue_size=3, latch=True)
    lmapfyPub = rospy.Publisher(rospy.get_name() + "/idealLeft/floatY", Image, queue_size=3, latch=True)

    rmapixPub=rospy.Publisher(rospy.get_name()+"/idealRight/intX",Image,queue_size=3,latch=True)
    rmapiyPub = rospy.Publisher(rospy.get_name() + "/idealRight/intY", Image, queue_size=3, latch=True)
    rmapfxPub = rospy.Publisher(rospy.get_name() + "/idealRight/floatX", Image, queue_size=3, latch=True)
    rmapfyPub = rospy.Publisher(rospy.get_name() + "/idealRight/floatY", Image, queue_size=3, latch=True)            

    cameraSettings["roi_width"]=cameraSettings["lInfo"].roi.width
    cameraSettings["roi_height"]=cameraSettings["lInfo"].roi.height
    cameraSettings["roi_x"]=cameraSettings["lInfo"].roi.x_offset
    cameraSettings["roi_y"]=cameraSettings["lInfo"].roi.y_offset
    cameraSettings["roi"]=[cameraSettings["roi_y"],cameraSettings["roi_x"],cameraSettings["roi_height"],cameraSettings["roi_width"]]

    cameraSettings["width"]=cameraSettings["lInfo"].width
    cameraSettings["height"]=cameraSettings["lInfo"].height
    cameraSettings["f"]=cameraSettings["Pl"][0,0]
    cameraSettings["pp"]=(cameraSettings["Pl"][0:2,2][0],
                        cameraSettings["Pl"][0:2,2][1])
    cameraSettings["k"]=cameraSettings["Pl"][0:3,0:3]

    cameraSettings["lMapix"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealLeft/intX",Image))
    cameraSettings["lMapiy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealLeft/intY",Image))
    cameraSettings["lMapfx"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealLeft/floatX",Image))
    cameraSettings["lMapfy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealLeft/floatY",Image))


    cameraSettings["rMapix"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealRight/intX",Image))
    cameraSettings["rMapiy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealRight/intY",Image))
    cameraSettings["rMapfx"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealRight/floatX",Image))
    cameraSettings["rMapfy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/idealRight/floatY",Image))
    print("Loaded")
    return cameraSettings


class stereoCamera:
    def __init__(self,configurationTopic=""):
        if(configurationTopic==""):
            self.kSettings=getCameraSettingsFromServer()
        else:
            self.kSettings=getCameraSettingsFromServer(configurationTopic)
    def debayerImage(self,bumblebeeImage):
        totalTime=0
        startTime=time.time()
        limage=copy.deepcopy(bumblebeeImage[self.kSettings["height"]:self.kSettings["height"]*2,0:self.kSettings["width"]])
        startTime=time.time()
        limage=cv2.cvtColor(limage,cv2.COLOR_BAYER_BG2GRAY)
        diff=time.time()-startTime
        totalTime+=diff
        
        rimage=copy.deepcopy(bumblebeeImage[0:self.kSettings["height"],0:self.kSettings["width"]])
        startTime=time.time()
        rimage=cv2.cvtColor(rimage,cv2.COLOR_BAYER_BG2GRAY)
        diff=time.time()-startTime
        totalTime+=diff
        return copy.deepcopy(limage),copy.deepcopy(rimage),totalTime
    def rectifyImage(self,leftImg,rightImg,floatingRect=True):
        totalTime=0
        if(floatingRect):
            startTime=time.time()
            lr=copy.deepcopy(cv2.remap(leftImg,
                                self.kSettings["lMapfx"],self.kSettings["lMapfy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff

            startTime=time.time()
            rr=copy.deepcopy(cv2.remap(rightImg,
                                self.kSettings["rMapfx"],self.kSettings["rMapfy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff
        else:
            startTime=time.time()
            lr=copy.deepcopy(cv2.remap(leftImg,
                                self.kSettings["lMapix"],self.kSettings["lMapiy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff

            startTime=time.time()
            rr=copy.deepcopy(cv2.remap(rightImg,
                                self.kSettings["rMapix"],self.kSettings["rMapiy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff
        return  lr,rr,totalTime
    # def getROIimg(self,img):
    #     return copy.deepcopy(img[(self.kSettings["roi"][1]:(self.kSettings["roi"][1]+self.kSettings["roi"][2]),
    #                               (self.kSettings["roi"][0]+self.ROI[1][3]:(self.kSettings["roi"][1]+self.kSettings["roi"][2])  ]


        # limages.append(copy.deepcopy(limages[3][self.ROI[0][1]:(self.ROI[0][1]+self.ROI[0][2]),self.ROI[0][0]:(self.ROI[0][0]+self.ROI[0][3])]))
        # rimages.append(copy.deepcopy(rimages[3][self.ROI[1][1]:(self.ROI[1][1]+self.ROI[1][2]),self.ROI[1][0]:(self.ROI[1][0]+self.ROI[1][3])]))#offreply.right.y_offset:offreply.right.height, offreply.right.x_offset:offreply.right.width]))
        # imageEncoding.append("mono8")
