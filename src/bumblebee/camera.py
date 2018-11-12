
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


def predictPoint(X,P,homogenous=True):
    '''
    Given a projective matrix,
    predict the pixel locations in ohomgenous coordinates

    assumes X is either a single [4xN,1] where [X1 Y1 Z1 W1 X2 Y2 ...]^T
    else it must be in form [4,N] where
            [X1 X2 X3 ... Xn]
    x{      [Y1 Y2 Y3 ... Yn]
            [Z1 Z2 Z3 ... Zn]
            [W1 W2 W3 ... Wn]
    returns M=P*x
    '''
    result=None
    if(X.shape[1]>=1):
        ###assume it is in the form
        ### 
        result=P.dot(X)  
    else:
        reshapedX=X.reshape((4,X.shape[1]/4))  
        result=P.dot(reshapedX)
    if(homogenous):
        result/=result[3,:]
    return result  


def composeCamera(K,xMotion=np.zeros((6,1))):
    P=np.zeros((3,4),dtype=np.float64)
    transform=motionEdge(xMotion[0:3].reshape(3),xMotion[3:6].reshape(3),degrees=False)
    P=K.dot(transform.getH()[0:3,:])
    return P

def reproject2(l,r,Q):
    dispVect=np.ones((4,1),dtype=np.float64)
    disparity=l[0,0]-r[0,0]
    dispVect[0,0]=l[0,0]
    dispVect[1,0]=l[1,0]
    dispVect[2,0]=disparity
    xPred=Q.dot(dispVect)
    return xPred/xPred[3,0]

def predictPoint(X,P):
    prediction=P.dot(X)
    prediction/=prediction[2]
    return prediction

def getPredictedDiff(Pl,Pr,X,l,r):
    predictL=Pl.dot(X)
    predictL/=predictL[2,0]
    predictR=Pr.dot(X)
    predictR/=predictR[2,0]

    return (predictL-l)[0:2,0],(predictR-r)[0:2,0]

