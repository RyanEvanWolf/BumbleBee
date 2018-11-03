 
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
from bumblebee.stereo import *
from bumblebee.motion import *
from bumblebee.baseTypes import *

def noisyRotations(noise=5):
    out=np.zeros((1,3))
    out[0,0]=np.random.normal(0,noise,1)
    out[0,1]=np.random.normal(0,noise,1)
    out[0,2]=np.random.normal(0,noise,1)
    return out

def forwardTranslation(zBase=0.2,noise=0.1):
    out=np.zeros((3,1))
    out[0,0]=np.random.normal(0,noise,1)
    out[1,0]=np.random.normal(0,noise,1)
    out[2,0]=abs(np.random.normal(zBase,noise,1))
    return out

def genRandomCoordinate(xAvg,yAvg,zAvg):
    Point=np.ones((4,1),dtype=np.float64)
    Point[0,0]=np.random.normal(0,xAvg,1)
    Point[1,0]=np.random.normal(0,yAvg,1)
    Point[2,0]=np.random.normal(0,zAvg,1)
    return Point
