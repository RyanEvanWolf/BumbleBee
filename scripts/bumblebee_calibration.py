#!/usr/bin/env python
import numpy as np
from sensor_msgs.msg import CameraInfo,RegionOfInterest
###
#single location for all standard required classes and data structures

class RectificationInfo():
    def __init__(self):
        self.intXMapping=None
        self.intYMapping=None
        self.floatXMapping=None
        self.floatYMapping=None


def generateBoardCoordinates():
    ###create a single chessboard coordinates assuming 1 point starts at 0
    outputCoordinates=[]
    for w in range(0,nWidth-1):
        for h in range(0,nHeight-1):
            boardCoordinate=[w*squareWidth,h*squareWidth,0]
            outputCoordinates.append(boardCoordinate)
    return outputCoordinates

class calibrationMetaData():
    def __init__(self,vArgs):
        self.inDirectory=vArgs[1]
        self.outDirectory=vArgs[2]
        self.squareWidth=float(vArgs[3])
        self.nWidth=int(vArgs[4])
        self.nHeight=int(vArgs[5])
        self.debayer=bool(vArgs[6])
        self.distortionModel=vArgs[7]
        self.imgSize=(0,0)
    def printSettings(self):
        print("Settings:")
        print(self.inDirectory)
        print(self.outDirectory)
        print(self.squareWidth)
        print(self.nWidth)
        print(self.nHeight)
        print(self.debayer)
        print(self.distortionModel)
class SingleCameraCalData():
    def __init__(self):
        self.Lparam=SingleCameraParamData()
        self.Rparam=SingleCameraParamData()
        self.meta=None
        self.LeftPoints=[]
        self.LeftReprojected = []
        self.PatternPoints=[]##the coordinates of each point relative to the checkerboard (0,0,0),(squareSize.0,0)....
        self.LT=[]
        self.LR=[]
        self.RR=[]
        self.RT=[]
        self.RightPoints=[]
        self.RightReprojected=[]
    def getRMSreprojection(self):
        avgErrorLeft=0
        avgErrorRight=0
        for imageIndex in range(len(self.LeftPoints)):
            for ptIndex in range(len(self.LeftPoints[imageIndex])):
                avgErrorLeft+=np.square(self.LeftPoints[imageIndex][ptIndex]-self.LeftReprojected[imageIndex][ptIndex]).sum()
        for imageIndex in range(len(self.RightPoints)):
            for ptIndex in range(len(self.RightPoints[imageIndex])):
                avgErrorRight+=np.square(self.RightPoints[imageIndex][ptIndex]-self.RightReprojected[imageIndex][ptIndex]).sum()
        npoints=len(self.LeftPoints)*len(self.LeftPoints[0])
        return [np.sqrt(avgErrorLeft/npoints),np.sqrt(avgErrorRight/npoints)]
    def getSingleImageRMS(self,imgIndex):
        leftImageRMS=0
        rightImageRMS=0
        for ptIndex in range(len(self.LeftPoints[imgIndex])):
            rightImageRMS += np.square(self.RightPoints[imgIndex][ptIndex] - self.RightReprojected[imgIndex][ptIndex]).sum()
            leftImageRMS += np.square(self.LeftPoints[imgIndex][ptIndex] - self.LeftReprojected[imgIndex][ptIndex]).sum()
        return [np.sqrt(leftImageRMS/len(self.LeftPoints[imgIndex])),np.sqrt(rightImageRMS/len(self.RightPoints[imgIndex]))]

class SingleCameraParamData():
    def __init__(self):
        self.K=None
        self.D=None
        self.RMS_error=0
        self.calibrationFlags=0