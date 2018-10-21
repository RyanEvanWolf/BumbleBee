 
import cv2
import math
import time
import random
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
import copy
from bumblebee.utils import *
from bumblebee.drawing import *

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
    cameraSettings["roi"]=[cameraSettings["roi_x"],cameraSettings["roi_y"],cameraSettings["roi_width"],cameraSettings["roi_height"]]

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

def checkWithinROI(pt,roi):
    ###
    ##check width
    if((pt[0,0]>=roi[0])and(pt[0,0]<=(roi[0]+roi[2]))):
        if((pt[1,0]>=roi[1])and(pt[1,0]<=(roi[1]+roi[3]))):   
            return True
        else:
            return False
    else:
        return False


def validateLandmark(data,Pl,Pr,roi,maxHeight,homography=np.eye(4)):
    ############
    ###transform into the new coordinate system
    xNew=homography.dot(data)
    if((xNew[1,0]<=maxHeight)and(xNew[2,0]>0)):
        l=Pl.dot(xNew)
        l/=l[2,0]
        r=Pr.dot(xNew)
        r/=r[2,0]
        ################################
        ##########check within ROI
        ###############################
        if(checkWithinROI(l,roi) and checkWithinROI(r,roi)):
            #########################
            ######generate gaussian data
            return True,l,r
        else:
            return False,None,None
    else:
        return False,None,None


def genRandomCoordinate(xAvg,yAvg,zAvg):
    Point=np.ones((4,1),dtype=np.float64)
    Point[0,0]=np.random.normal(0,xAvg,1)
    Point[1,0]=np.random.normal(0,yAvg,1)
    Point[2,0]=np.random.normal(0,zAvg,1)
    return Point

def genRandomGaussian(lPixel,rPixel,kSettings,noisePixels):
    leftNoise=copy.deepcopy(lPixel)
    rightNoise=copy.deepcopy(rPixel)
    leftNoise[0,0]+=np.random.normal(0,noisePixels,1)
    leftNoise[1,0]+=np.random.uniform()
    rightNoise[0,0]+=np.random.normal(0,noisePixels,1)
    rightNoise[1,0]=copy.deepcopy(leftNoise[1,0])###ensure still valid correspondence
    xNoise=cv2.triangulatePoints(kSettings["Pl"],
                                kSettings["Pr"],
                                (leftNoise[0,0],leftNoise[1,0]),
                                (rightNoise[0,0],rightNoise[1,0]))
    xNoise/=xNoise[3,0]
    return xNoise,leftNoise,rightNoise  

def genRandomSimulationPoint(kSettings,landmarkSettings,H=np.eye(4)):
    validPoint=False
    Data={}
    Data["Noise"]={}
    Data["Ideal"]={}
    while(not validPoint):
        Point=genRandomCoordinate(landmarkSettings["Xdepth"],
                                    landmarkSettings["Ydepth"],
                                    landmarkSettings["Zdepth"])
        validPoint,l,r=validateLandmark(Point,
                                    kSettings["Pl"],
                                    kSettings["Pr"],
                                    kSettings["roi"],
                                    landmarkSettings["HeightMaximum"])
        Data["Ideal"]["X"]=Point
        Data["Ideal"]["L"]=l
        Data["Ideal"]["R"]=r
    Data["Ideal"]["Xpred"]=cv2.triangulatePoints(kSettings["Pl"],
                            kSettings["Pr"],
                            (l[0,0],l[1,0]),
                            (r[0,0],r[1,0]))
    Data["Ideal"]["Xpred"]/=Data["Ideal"]["Xpred"][3,0]
    Data["Ideal"]["XPredRMS"]=np.linalg.norm(Point-Data["Ideal"]["Xpred"])
    #########
    ##gen noisy data
    for noise in landmarkSettings["GaussianNoise"]:
        validNoise=False
        NoiseData={}
        while(not validNoise):
            xtest,ltest,rtest=genRandomGaussian(Data["Ideal"]["L"],Data["Ideal"]["R"],kSettings,noise)
            validNoise,l,r=validateLandmark(xtest,
                                    kSettings["Pl"],
                                    kSettings["Pr"],
                                    kSettings["roi"],
                                    landmarkSettings["HeightMaximum"])
            NoiseData["X"]=xtest
            NoiseData["L"]=l
            NoiseData["R"]=r
            NoiseData["Linitial"]=ltest
            NoiseData["Rinitial"]=rtest
        name=str(noise).replace(".","_")
        Data["Noise"][name]=NoiseData
    #########
    ##gen outlier value
    ########
    validOutlier=False
    while(not validOutlier):
        Point=genRandomCoordinate(landmarkSettings["Xdepth"],
                                    landmarkSettings["Ydepth"],
                                    landmarkSettings["Zdepth"])
        inROI,l,r=validateLandmark(Point,
                                    kSettings["Pl"],
                                    kSettings["Pr"],
                                    kSettings["roi"],
                                    landmarkSettings["HeightMaximum"])
        if(inROI and (sumSquareError(l,Data["Ideal"]["L"])>=landmarkSettings["MinimumOutlier"])
            and(sumSquareError(r,Data["Ideal"]["R"])>=landmarkSettings["MinimumOutlier"])):
            validOutlier=True
            Data["Outlier"]={}
            Data["Outlier"]["X"]=Point
            Data["Outlier"]["L"]=l
            Data["Outlier"]["R"]=r
    return Data

class Landmark:
    def __init__(self):
        self.edges=[]

# class motionEdge:
#     def __init__(self):
#         self.angles=None
#         self.Tc=None
#         self.R=None
#         self.H=None
#         self.Htransform=None
#     def initH(self,H):
#         self.H=copy.deepcopy(H)
#         self.R=copy.deepcopy(H[0:3,0:3])
#         tmpAngles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
#         self.angles=np.zeros((1,3))
#         self.angles[0,0]=degrees(tmpAngles[0])
#         self.angles[0,1]=degrees(tmpAngles[1])
#         self.angles[0,2]=degrees(tmpAngles[2])

#         self.Tc=np.zeros((3,1))
#         self.Tc[0,0]=self.H[0,3]
#         self.Tc[1,0]=self.H[1,3]
#         self.Tc[2,0]=self.H[2,3]
#         self.Htransform=composeTransform(self.R,self.Tc)
#     def initData(self,angles=np.zeros((1,3)),coords=np.zeros((3,1))):
#         self.angles=copy.deepcopy(angles) ###in degrees
#         self.Tc=copy.deepcopy(coords)  ###in meters
#         self.R=composeR(self.angles[0,0],self.angles[0,1],self.angles[0,2])##assume degrees
#         self.H=createHomog(self.R,self.Tc)
#         self.Htransform=composeTransform(self.R,self.Tc)
#     def getMotion(self):
#         dictionary={}
#         return dictionary
  


# def getMotion(H):
# Result={}
# angles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
# Result["Roll"]=57.2958*angles[0]
# Result["Pitch"]=57.2958*angles[1]
# Result["Yaw"]=57.2958*angles[2]

# Result["X"]=copy.deepcopy(H[0,3])
# Result["Y"]=copy.deepcopy(H[1,3])
# Result["Z"]=copy.deepcopy(H[2,3])
# return Result
        #self.H=copy.deepcopy(H)
#         simulationData["R"]=noisyRotations(mSettings[m]["RotationNoise"])
#         simulationData["Tc"]=dominantTranslation(mSettings[m]["TranslationMean"],mSettings[m]["TranslationNoise"])
#         simulationData["H"]=createHomog(simulationData["R"]["matrix"],
#                                         simulationData["Tc"]["vector"])
#         simulationData["Htransform"]=composeTransform(simulationData["R"]["matrix"],
#                                         simulationData["Tc"]["vector"])

class stereoLandmark:
    def __init__(self,X,L,R):
        self.X=None     ####np x,y,z,w
        self.L=None     ###np, x,y,w
        self.R=None     ###np,x,y,w
        ####always includes offset in point, no ROI
    def getTriangulateError(self,useQ=False):
        pass
    def getReprojectError(self,Pl,Pr):
        pass


class stereoFrame:
    def __init__(self):
        self.landmarks=[]   ###

class stereoWindow:
    def __init__(self):
        self.stereoFrames=[]  ####oldest - > newest (FIFO)

class simulationFrame:
    def __init__(self):
        self.H=None
        self.landmarks=[]
    def getLeftPoints(self):
        pass
    def getRightPoints(self):
        pass
    def getXPoints(self):
        pass
    def drawEpiMatches(self,width,height,roi):
        lImage=255*np.ones((height,width,3),dtype=np.uint8)
        rImage=255*np.ones((height,width,3),dtype=np.uint8)
        displayOffset=np.zeros((3,1))
        displayOffset[0,0]=width
        for i in self.landmarks:
            cv2.circle(lImage,np2cvDisplay(i["Ideal"]["L"]),2,(255,0,0,0),-1)
            cv2.circle(rImage,np2cvDisplay(i["Ideal"]["R"]),2,(0,0,255,0),-1)
            cv2.line(lImage,np2cvDisplay(i["Ideal"]["L"]),(width,np2cvDisplay(i["Ideal"]["L"])[1]),(1,0,0,0))
            cv2.line(rImage,np2cvDisplay(i["Ideal"]["R"]),(0,np2cvDisplay(i["Ideal"]["R"])[1]),(1,0,0,0))
        epiImage=np.hstack((lImage,rImage))
        return drawStereoROI(epiImage,roi,roi)
    def drawOutlier(self,width,height,roi):
        lImage=255*np.ones((height,width,3),dtype=np.uint8)
        # rImage=255*np.ones((height,width,3),dtype=np.uint8)
        displayOffset=np.zeros((3,1))
        displayOffset[0,0]=width
        for i in self.landmarks:
            cv2.circle(lImage,np2cvDisplay(i["Ideal"]["L"]),2,(0,0,0,1),-1)
            cv2.circle(lImage,np2cvDisplay(i["Outlier"]["L"]),2,(255,0,0,1),-1)
            cv2.line(lImage,np2cvDisplay(i["Ideal"]["L"]),np2cvDisplay(i["Outlier"]["L"]),(255,0,0,1))
        return lImage     
    def drawGaussian(self,width,height,roi):
        lImage=255*np.ones((height,width,3),dtype=np.uint8)
        displayOffset=np.zeros((3,1))
        displayOffset[0,0]=width
        for i in self.landmarks:
            currentPoint=np2cvDisplay(i["Ideal"]["L"])
            cv2.circle(lImage,currentPoint,2,(0,0,0,1),-1)
            for j in i["Noise"].keys():
                cv2.line(lImage,currentPoint,np2cvDisplay(i["Noise"][j]["L"]),(255,0,0,1))
        return lImage           

class stereoCamera:
    def __init__(self,configurationTopic=""):
        if(configurationTopic==""):
            self.kSettings=getCameraSettingsFromServer()
        else:
            self.kSettings=getCameraSettingsFromServer(configurationTopic)
    def simulateFrame(self,simSettings,nPoints=10):
        outFrame=simulationFrame()
        outFrame.H=np.eye(4)
        for pointID in range(0,nPoints):
            outFrame.landmarks.append(genRandomSimulationPoint(self.kSettings,simSettings))
        return outFrame
    def checkWithinROI(self,pt):
        return checkWithinROI(pt,self.kSettings["roi"])
    def simulateMotionFrame(self,simSettings,H,nPoints=10):
        #####Gen normal landmark
        outFrame=simulationFrame()
        for pointIndex in range(0,nPoints):
            validSimulation=False
            while(not validSimulation):
                landmarkA=genRandomSimulationPoint(self.kSettings,simSettings)
                validTransform,l,r=validateLandmark(landmarkA["Ideal"]["X"],
                                    self.kSettings["Pl"],
                                    self.kSettings["Pr"],
                                    self.kSettings["roi"],
                                    simSettings["HeightMaximum"],H)
                if(validTransform):
                    reproject1=self.reproject(l,r)
                    rp2=self.reproject2(l,r)
                    x1=rmsError(np.linalg.inv(H).dot(reproject1),landmarkA["Ideal"]["X"])
                    x2=rmsError(np.linalg.inv(H).dot(rp2),landmarkA["Ideal"]["X"])
                    print(np.transpose(x1),np.transpose(x2))
                    print(np.transpose(np.linalg.inv(H).dot(reproject1)),
                          np.transpose(np.linalg.inv(H).dot(rp2)),
                            np.transpose(landmarkA["Ideal"]["X"]))
                    print("---")
                    validSimulation=True
                    outFrame.landmarks.append(landmarkA)
        return outFrame
# def genRandomSimulationPoint(kSettings,landmarkSettings,debug=False):
#     validPoint=False
#     Data={}
#     Data["Noise"]={}
#     Data["Ideal"]={}
#     while(not validPoint):
#         Point=genRandomCoordinate(landmarkSettings["Xdepth"],
#                                     landmarkSettings["Ydepth"],
#                                     landmarkSettings["Zdepth"])
#         validPoint,l,r=validateLandmark(Point,
#                                     kSettings["Pl"],
#                                     kSettings["Pr"],
#                                     kSettings["roi"],
#                                     landmarkSettings["HeightMaximum"])
#         Data["Ideal"]["X"]=Point
#         Data["Ideal"]["L"]=l
#         Data["Ideal"]["R"]=r
#     Data["Ideal"]["Xpred"]=cv2.triangulatePoints(kSettings["Pl"],
#                             kSettings["Pr"],
#                             (l[0,0],l[1,0]),
#                             (r[0,0],r[1,0]))
#     Data["Ideal"]["Xpred"]/=Data["Ideal"]["Xpred"][3,0]
#     Data["Ideal"]["XPredRMS"]=np.linalg.norm(Point-Data["Ideal"]["Xpred"])
#     #########
#     ##gen noisy data
#     for noise in landmarkSettings["GaussianNoise"]:
#         validNoise=False
#         NoiseData={}
#         while(not validNoise):
#             xtest,ltest,rtest=genRandomGaussian(Data["Ideal"]["L"],Data["Ideal"]["R"],kSettings,noise)
#             validNoise,l,r=validateLandmark(xtest,
#                                     kSettings["Pl"],
#                                     kSettings["Pr"],
#                                     kSettings["roi"],
#                                     landmarkSettings["HeightMaximum"])
#             NoiseData["X"]=xtest
#             NoiseData["L"]=l
#             NoiseData["R"]=r
#             NoiseData["Linitial"]=ltest
#             NoiseData["Rinitial"]=rtest
#         name=str(noise).replace(".","_")
#         Data["Noise"][name]=NoiseData
#     #########
#     ##gen outlier value
#     ########
#     validOutlier=False
#     while(not validOutlier):
#         Point=genRandomCoordinate(landmarkSettings["Xdepth"],
#                                     landmarkSettings["Ydepth"],
#                                     landmarkSettings["Zdepth"])
#         inROI,l,r=validateLandmark(Point,
#                                     kSettings["Pl"],
#                                     kSettings["Pr"],
#                                     kSettings["roi"],
#                                     landmarkSettings["HeightMaximum"])
#         if(inROI and (sumSquareError(l,Data["Ideal"]["L"])>=landmarkSettings["MinimumOutlier"])
#             and(sumSquareError(r,Data["Ideal"]["R"])>=landmarkSettings["MinimumOutlier"])):
#             validOutlier=True
#             Data["Outlier"]={}
#             Data["Outlier"]["X"]=Point
#             Data["Outlier"]["L"]=l
#             Data["Outlier"]["R"]=r
#     return Data            
        #####transform it
        #####   check it still fits in ROI
        #####DO this N times
        ####draw the simulated images
        ####return the data.
        pass
    def predictPoint(self,Xworld):
        leftPixel=self.kSettings["Pl"].dot(Xworld)
        leftPixel/=leftPixel[2,0]
        rightPixel=self.kSettings["Pr"].dot(Xworld)
        rightPixel/=rightPixel[2,0]
        return leftPixel,rightPixel 
    def reproject(self,l,r):
        xPred=cv2.triangulatePoints(self.kSettings["Pl"],
                                self.kSettings["Pr"],
                                (l[0,0],l[1,0]),
                                (r[0,0],r[1,0]))   
        
        return xPred/xPred[3,0]
    def reproject2(self,l,r):
        dispVect=np.ones((4,1),dtype=np.float64)
        disparity=l[0,0]-r[0,0]
        dispVect[0,0]=l[0,0]
        dispVect[1,0]=l[1,0]
        dispVect[2,0]=disparity
        xPred=self.kSettings["Q"].dot(dispVect)
        return xPred/xPred[3,0]
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
