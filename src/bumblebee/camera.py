
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


def reproject2(l,r,Q):
    dispVect=np.ones((4,1),dtype=np.float64)
    disparity=l[0,0]-r[0,0]
    dispVect[0,0]=l[0,0]
    dispVect[1,0]=l[1,0]
    dispVect[2,0]=disparity
    xPred=Q.dot(dispVect)
    return xPred/xPred[3,0]

def PredictPoint(X,P):
    prediction=P.dot(X)
    prediction/=prediction[2]
    return prediction

def getPredictedDiff(Pl,Pr,X,l,r):
    predictL=Pl.dot(X)
    predictL/=predictL[2,0]
    predictR=Pr.dot(X)
    predictR/=predictR[2,0]

    return (predictL-l)[0:2,0],(predictR-r)[0:2,0]

