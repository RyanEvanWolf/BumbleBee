 
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
from bumblebee.motion import *
from bumblebee.camera import *
import json

class stereoLandmarkEdge:
    def __init__(self,L,R,X=None,ID=-1):
        self.L=copy.deepcopy(L)
        self.R=copy.deepcopy(R)
        self.X=copy.deepcopy(X)
        self.ID=copy.deepcopy(ID)
    def getDisparity(self):
        return self.L[0,0]-self.R[0,0]
    def getEpiError(self):
        return self.L[1,0]-self.R[1,0]
    def getQtriangulate(self,Q):
        return reproject2(self.L,self.R,Q)
    def getReprojectedDifference(self,Pl,Pr):
        return getPredictedDiff(Pl,Pr,self.X,self.L,self.r)


class stereoFrame:
    def __init__(self):
        self.X=None
        self.R=None
        self.L=None

# class stereoFrame:
#     def __init__(self,lEdges=None,rEdges=None):
#         self.KPl=None
#         self.KPr=None
#         self.rawLdesc=None
#         self.rawRdesc=None
#         self.X=None
#         self.L=None
#         self.R=None
      




class stereoEdge:
    def __init__(self,X,L,R,frameNumber=-1):
        self.X=copy.deepcopy(X)
        self.L=copy.deepcopy(L)
        self.R=copy.deepcopy(R)
        self.frameID=frameNumber
    def __str__(self):
        ans=""
        ans+="ID:"+str(self.frameID)+"\n"
        ans+="X:"+str(np.transpose(self.X))+"\n"
        ans+="L:"+str(np.transpose(self.L))+"\n"
        ans+="R:"+str(np.transpose(self.R))+"\n"
        return ans
class trackEdge:
    def __init__(self):
        self.tracks=[]
        self.motion=[]
    
class interFrameEdge:
    def __init__(self):
        self.currentEdges=[]
        self.Tracks=[]
        self.previousEdges=[]
        self.motion=None
    def getCurrentX(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.X)
        return result
    def getPreviousX(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.X)
        return result   
    def getPreviousL(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.L)
        return result      
    def getPreviousR(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.R)
        return result        
    def getCurrentL(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.L)
        return result      
    def getCurrentR(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.R)
        return result     


# class motionEdge:
#     def __init__(self,H):
#         self.H=copy.deepcopy(H)
#         self.angles=None
#         self.Tc=None
#         self.R=None
#         self.H=None
#         self.Htransform=None
#     def getR(self):
#         return copy.deepcopy(self.H[0:3,0:3])
#     def getT(self):
#         return copy.deepcopy(self)
#     def getCoordinateTransform(self):
#         return composeTransform()
#     def invert(self,H=None):
#         if(H==None):
#             self.H=np.linalg.inv(self.H.dot(H)

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
