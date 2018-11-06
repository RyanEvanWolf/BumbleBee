
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

