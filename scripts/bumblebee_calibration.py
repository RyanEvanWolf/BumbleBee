#!/usr/bin/env python
from sensor_msgs.msg import CameraInfo,RegionOfInterest
###
#single location for all standard required classes and data structures

class RectificationInfo():
    def __init__(self):
        self.intXMapping=None
        self.intYMapping=None
        self.floatXMapping=None
        self.floatYMapping=None

class SingleCameraCalData():
    def __init__(self):
        self.param=SingleCameraParamData()
        self.inDirectory=None
        self.maskedImages=[]
        self.name = None
        self.LeftPoints=[]
        self.LeftPatternPoints=[]##the coordinates of each point relative to the checkerboard (0,0,0),(squareSize.0,0)....
        self.LeftReprojected=[]
        self.RightPoints=[]
        self.RightReprojected=[]
        self.RightPatternPoints=[]
        self.squareWidth=None
        self.nWidth=None
        self.nHeight=None
        self.debayer=None
        self.imgSize=None
        print("squareWidth", squareWidth)
        print("nWidth", nWidth)
        print("nHeight", nHeight)
        print("distortModel", distortModel)
        print("debayer", debayer)
class SingleCameraParamData():
    def __init__(self):
        self.K=None
        self.D=None
        self.distortionModel=""
        self.RMS_error=0