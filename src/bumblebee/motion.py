import numpy as np
from tf.transformations import quaternion_from_euler,quaternion_matrix,euler_from_matrix
from math import pi,radians,degrees
import rosbag
import time
import cv2
import copy
import decimal




def composeR(Rtheta,degrees=False):
    if(degrees):
        q=quaternion_from_euler(radians(Rtheta[0]),
                                radians(Rtheta[1]),
                                radians(Rtheta[2]),'szxy')
    else:
        q=quaternion_from_euler(Rtheta[0],
                                Rtheta[1],
                                Rtheta[2],'szxy')     
    return quaternion_matrix(q)[0:3,0:3]  

    
def getDNister():
    out=np.zeros((3,3),dtype=np.float64)
    out[0,1]=1
    out[1,0]=-1
    out[2,2]=1
    return out


def getHomogZeros():
    out=np.zeros((4,1),dtype=np.float64)
    out[3,0]=1
    return out

def createHomog(R=np.eye(3,dtype=np.float64),
                T=np.zeros((3,1),np.float64)):
    output=np.eye(4,dtype=np.float64)
    output[0:3,0:3]=R
    output[0:3,3]=T.reshape(3)
    return output

def composeTransform(R,T):
    ####H=[R -RT]
    ######[0   1]
    return createHomog(R,-R.dot(T))


def decomposeTransform(H):
    R=copy.deepcopy(H[0:3,0:3])
    T=-1*np.linalg.inv(R).dot(H[0:3,3])
    return createHomog(R,T)


def getMotion(H):
    Result={}
    angles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
    Result["Roll"]=57.2958*angles[0]
    Result["Pitch"]=57.2958*angles[1]
    Result["Yaw"]=57.2958*angles[2]

    Result["X"]=copy.deepcopy(H[0,3])
    Result["Y"]=copy.deepcopy(H[1,3])
    Result["Z"]=copy.deepcopy(H[2,3])
    return Result

def getRotationMotion(R):
    Result={}
    angles=copy.deepcopy(euler_from_matrix(R,'szxy'))
    Result["Roll"]=57.2958*angles[0]
    Result["Pitch"]=57.2958*angles[1]
    Result["Yaw"]=57.2958*angles[2]
    return Result

def compareMotion(H,Hest):
    Hstruct=getMotion(H)
    HestStruct=getMotion(Hest)
    Result={}
    Result["Roll"]=getPercentError(Hstruct["Roll"],HestStruct["Roll"])
    Result["Pitch"]=getPercentError(Hstruct["Pitch"],HestStruct["Pitch"])
    Result["Yaw"]=getPercentError(Hstruct["Yaw"],HestStruct["Yaw"])
    Result["X"]=getPercentError(Hstruct["X"],HestStruct["X"])
    Result["Y"]=getPercentError(Hstruct["Y"],HestStruct["Y"])
    Result["Z"]=getPercentError(Hstruct["Z"],HestStruct["Z"])
    return Result

def compareAbsoluteMotion(H,Hest):
    Hstruct=getMotion(H)
    HestStruct=getMotion(Hest)
    Result={}
    Result["Roll"]=round(getAbsoluteError(Hstruct["Roll"],HestStruct["Roll"]),3)
    Result["Pitch"]=round(getAbsoluteError(Hstruct["Pitch"],HestStruct["Pitch"]),3)
    Result["Yaw"]=round(getAbsoluteError(Hstruct["Yaw"],HestStruct["Yaw"]),3)
    Result["X"]=round(getAbsoluteError(Hstruct["X"],HestStruct["X"])*1000,3)###in mm
    Result["Y"]=round(getAbsoluteError(Hstruct["Y"],HestStruct["Y"])*1000,3)
    Result["Z"]=round(getAbsoluteError(Hstruct["Z"],HestStruct["Z"])*1000,3)
    return Result   

def compareUnitMotion(H,Hest):
    ###normalize the original to unit
    Hstruct=getMotion(H)
    HestStruct=getMotion(Hest)  

    structNorm=np.sqrt(Hstruct["X"]**2 + Hstruct["Y"]**2 + Hstruct["Z"]**2)
    Hstruct["X"]/=structNorm
    Hstruct["Y"]/=structNorm
    Hstruct["Z"]/=structNorm
    Result={}
    Result["Roll"]=getPercentError(Hstruct["Roll"],HestStruct["Roll"])
    Result["Pitch"]=getPercentError(Hstruct["Pitch"],HestStruct["Pitch"])
    Result["Yaw"]=getPercentError(Hstruct["Yaw"],HestStruct["Yaw"])
    Result["X"]=getPercentError(Hstruct["X"],HestStruct["X"])
    Result["Y"]=getPercentError(Hstruct["Y"],HestStruct["Y"])
    Result["Z"]=getPercentError(Hstruct["Z"],HestStruct["Z"])
    print(np.sqrt(Hstruct["X"]**2+Hstruct["Y"]**2+Hstruct["Z"]),
            np.sqrt(HestStruct["X"]**2+HestStruct["Y"]**2+HestStruct["Z"]))
    return Result

def getUnitTranslation(H):
    T=copy.deepcopy(H[0:3,3])
    return T


def getAbsoluteError(ideal,measured):
    diff=abs(ideal-measured)
    return diff

def getPercentError(ideal,measured):
    diff=ideal-measured
    pError=100*diff/abs(float(ideal))
    return pError

def getPsuedoInverseColumn(T):
    return np.linalg.pinv(get3x3Translation(T))

def get3x3Translation(T):
    return T.dot(np.transpose(T.reshape(3,1)))

def estimateScale(xPrev,xCurrent,R,T,inliers):
    Rinv=R
    Ti=T
    averageScale=np.zeros((3,3),dtype=np.float64)
    countedIn=0
    Tinv=getPsuedoInverseColumn(Ti)
    #print("counted",list(inliers).count(255))
    for index in range(0,len(xCurrent)):
        if(inliers[index,0]>0):
            Xb=xCurrent[index][0:3,0].reshape(3,1)
            Xa=xPrev[index][0:3,0].reshape(3,1)
            scale=(Xb-R.dot(Xa)).dot(np.transpose(Ti.reshape(3,1))).dot(Tinv)
            averageScale+=scale 
            countedIn+=1
    averageScale=averageScale/countedIn
    T=averageScale.dot(Ti)
    return averageScale,T,countedIn


def getRtheta(H):
    return copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))

class motionEdge:
    def __init__(self,Rvect=None,Tvect=None,H=None,degrees=True):
        self.x=np.zeros((6,1))
        if((Rvect is not None) and (Tvect is not None)):
            if(degrees):
                self.x[0:3,0]=np.radians(Rvect).reshape(3)
            else:
                self.x[0:3,0]=Rvect
            self.x[3:,0]=Tvect.reshape(3)
        elif(H is not None):
            self.x[0:3,0]=getRtheta(H)
            self.x[3:,0]=H[0:3,3]
    def getR(self):
        return composeR(self.x[0:3,0])
    def getT(self):
        return copy.deepcopy(self.x[3:,0])
    def getH(self):
        return createHomog(self.getR(),self.getT())
    def getC(self):
        return -1*np.linalg.inv(self.getR()).dot(self.getT())
    def getRtheta(self):
        ans=np.zeros((3,1))
        ans[0]=degrees(self.x[0])
        ans[1]=degrees(self.x[1])
        ans[2]=degrees(self.x[2])
        return ans
    def getFormatted(self):
        result={}
        Rtheta=self.getRtheta()
        result["Roll"]=Rtheta[0,0]
        result["Pitch"]=Rtheta[1,0]
        result["Yaw"]=Rtheta[2,0]
        c=self.getC()
        result["X"]=1000*c[0]
        result["Y"]=1000*c[1]
        result["Z"]=1000*c[2]
        return result

