 
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

def getProjections(cameraSet,Xset):
    '''
    cameraSet  = [P0 P1  P2  P3]
    Xset= [X1 X2 X3]
    M=[P0.X1   P0.X2   
       P1.X1   P1.X2   
            ....    ]
    '''

    M=np.zeros((3*len(cameraSet)/2,Xset.shape[1]))
    for lIndex in range(Xset.shape[1]):
        currentX=Xset[:,lIndex].reshape(4,1)

        for kIndex in range(len(cameraSet)/2):
            predictionL=cameraSet[kIndex*2].dot(currentX)
            predictionL/=predictionL[2,0]
            predictionR=cameraSet[kIndex*2+1].dot(currentX)
            predictionR/=predictionR[2,0]  
            M[kIndex*3,lIndex]=predictionL[0,0]
            M[kIndex*3+1,lIndex]=predictionL[1,0]
            M[kIndex*3+2,lIndex]=predictionR[0,0]
     
    return M

def composeProjectionMatrix(K,R,T):
    #P=K[R|T]=K[R|-RC]
    P=np.zeros((3,4),dtype=np.float64)
    P[0:3,0:3]=R
    P[0:3,3]=T
    P = K.dot(P)
    return P 


def rmsReprojectionError(Pl,Pr,Xset,Hset,Mset):
    '''
    Xset=[Xn Xn+1 Xn+2 ...]
    Hset=[Hn to n+1     Hn+1 to Hn+2     ...]
    Mset=[Lu   ...
          Lv   ...
          Ru   ...
                        ]
    '''

    for landmark in range(0,len(Xset)):
        Ml=predictPoint(Xset[landmark],Pl)
        Mr=predictPoint(Xset[landmark],Pr)
        print(Ml.shape)
    print(len(Xset),len(Hset),len(Mset))




    return 0
    # Xest,lPixels,rPixels):


    #         leftPixel=self.kSettings["Pl"].dot(Xworld)
    #     leftPixel/=leftPixel[2,0]
    #     rightPixel=self.kSettings["Pr"].dot(Xworld)
    #     rightPixel/=rightPixel[2,0]
    #     return leftPixel,rightPixel 


def ROIfrmMsg(Msg):
    return (Msg.x_offset,Msg.y_offset,Msg.width,Msg.height)#x,y,w,h

def getRectificationMappings(serverNodeName="/bumblebee_configuration",left=True):
    cvb=CvBridge()
    cameraSettings={}
    if(left):
        cameraSettings["Mapix"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Left/intX",Image))
        cameraSettings["Mapiy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Left/intY",Image))
        cameraSettings["Mapfx"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Left/floatX",Image))
        cameraSettings["Mapfy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Left/floatY",Image))
    else:
        cameraSettings["Mapix"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Right/intX",Image))
        cameraSettings["Mapiy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Right/intY",Image))
        cameraSettings["Mapfx"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Right/floatX",Image))
        cameraSettings["Mapfy"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Right/floatY",Image))  
    return cameraSettings

def getCameraSettingsFromServer(serverNodeName="/bumblebee_configuration",cameraType="subROI",full=False):
    cvb=CvBridge()
    ##assumes a node has been declared
    cameraSettings={}
    cameraSettings["FULL"]=full
    print(serverNodeName,cameraType)
    cameraSettings["Q"]=cvb.imgmsg_to_cv2(rospy.wait_for_message(serverNodeName+"/Q",Image))
    if(full):
        cameraSettings["Lmap"]=getRectificationMappings()
        cameraSettings["Rmap"]=getRectificationMappings(left=False)
    print(serverNodeName+"/Left/"+cameraType+"/CameraInfo")
    print(serverNodeName+"/Right/"+cameraType+"/CameraInfo")
    cameraSettings["lInfo"]=rospy.wait_for_message(serverNodeName+"/Left/"+cameraType+"/CameraInfo",CameraInfo)
    cameraSettings["rInfo"]=rospy.wait_for_message(serverNodeName+"/Right/"+cameraType+"/CameraInfo",CameraInfo)
    cameraSettings["Pl"]=np.zeros((3,4),dtype=np.float64)
    cameraSettings["Pr"]=np.zeros((3,4),dtype=np.float64)
    for row in range(0,3):
            for col in range(0,4):
                cameraSettings["Pl"][row,col]=cameraSettings["lInfo"].P[row*4 +col]
                cameraSettings["Pr"][row,col]=cameraSettings["rInfo"].P[row*4 +col]


    # cameraSettings["roi_width"]=cameraSettings["lInfo"].roi.width
    # cameraSettings["roi_height"]=cameraSettings["lInfo"].roi.height
    # cameraSettings["roi_x"]=cameraSettings["lInfo"].roi.x_offset
    # cameraSettings["roi_y"]=cameraSettings["lInfo"].roi.y_offset
    # cameraSettings["roi"]=[cameraSettings["roi_x"],cameraSettings["roi_y"],cameraSettings["roi_width"],cameraSettings["roi_height"]]

    cameraSettings["width"]=cameraSettings["lInfo"].width
    cameraSettings["height"]=cameraSettings["lInfo"].height
    cameraSettings["f"]=cameraSettings["Pl"][0,0]
    cameraSettings["pp"]=(cameraSettings["Pl"][0:2,2][0],
                        cameraSettings["Pl"][0:2,2][1])
    cameraSettings["k"]=cameraSettings["Pl"][0:3,0:3]

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
    def __init__(self,cameraType="subROI",configurationTopic=""):
        self.kSettings={}
        if(configurationTopic==""):
            self.kSettings=getCameraSettingsFromServer(cameraType=cameraType)
        else:
            self.kSettings=getCameraSettingsFromServer(configurationTopic,cameraType)

    def checkWithinROI(self,pt,left=True):
        if(left):
            return checkWithinROI(pt,ROIfrmMsg(self.kSettings["lInfo"].roi))
        else:
            return checkWithinROI(pt,ROIfrmMsg(self.kSettings["rInfo"].roi))
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
                                self.kSettings["lMap"]["Mapfx"],self.kSettings["lMap"]["Mapfy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff

            startTime=time.time()
            rr=copy.deepcopy(cv2.remap(rightImg,
                                self.kSettings["rMap"]["Mapfx"],self.kSettings["rMap"]["Mapfy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff
        else:
            startTime=time.time()
            lr=copy.deepcopy(cv2.remap(leftImg,
                                self.kSettings["lMap"]["Mapix"],self.kSettings["lMap"]["Mapiy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff

            startTime=time.time()
            rr=copy.deepcopy(cv2.remap(rightImg,
                                self.kSettings["rMap"]["Mapix"],self.kSettings["rMap"]["Mapiy"],cv2.INTER_LINEAR))
            diff=time.time()-startTime
            totalTime+=diff
        return  lr,rr,totalTime

