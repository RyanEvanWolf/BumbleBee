#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

import scipy.stats.mstats as stat

import matplotlib.pyplot as plt
from bumblebee_calibration import *

import pickle
import copy
calibrationDirectory=sys.argv[1]
plt.style.use('seaborn')



print("Loading calibration data from "+calibrationDirectory+"/StereoCalibration.p")
calibrationData=pickle.load(open(calibrationDirectory+"/StereoCalibration.p", "rb" ))
print("file opened")
print("loading camera Information")
leftMapping = pickle.load(open(calibrationDirectory + "/left_Map.p", "rb"))
rightMapping = pickle.load(open(calibrationDirectory + "/right_Map.p", "rb"))
intrin=pickle.load(open(calibrationDirectory + "/intrinsicCalibration.p", "rb"))
extrin=pickle.load(open(calibrationDirectory + "/extrinsicCalibration.p", "rb"))
print("camera info Load")
##calculate epipolar error

leftRectified=[]
rightRectified=[]
leftDebayer=[]
rightDebayer=[]


##list of list of found points by image, then by checkerboard position
leftFoundPoints=[]
rightFoundPoints=[]
epiError=[]
epiPolarError=0


outWindow=cv2.namedWindow("outWindow",cv2.WINDOW_NORMAL)


outFolder=calibrationDirectory+"/Matches"

if(not os.path.isdir(outFolder)):
        os.makedirs(outFolder)

print("Rectifying Images")
totalImagesFound=0
for fileName in calibrationData.inCalibrationData.imgName:
    ##get images
    fullImage=cv2.imread(calibrationData.inCalibrationData.meta.inDirectory+"/"+fileName,cv2.IMREAD_GRAYSCALE)
    if(calibrationData.inCalibrationData.meta.debayer):
        fullImage=cv2.cvtColor(fullImage,cv2.COLOR_BAYER_BG2GRAY)
    leftImage = fullImage[fullImage.shape[0] / 2:fullImage.shape[0], 0:fullImage.shape[1]]
    leftDebayer.append(leftImage)
    rightImage = fullImage[0:fullImage.shape[0] / 2, 0:fullImage.shape[1]]
    rightDebayer.append(rightImage)
    ##rectify them
    leftRectified.append(leftMapping.getRectifiedImage(leftImage))
    rightRectified.append(rightMapping.getRectifiedImage(rightImage))

    lFound, lCorners = cv2.findChessboardCorners(leftRectified[-1], calibrationData.inCalibrationData.meta.getBoardSize(),
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    rFound, rCorners = cv2.findChessboardCorners(rightRectified[-1], calibrationData.inCalibrationData.meta.getBoardSize(),
                                                 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    winSize = (7, 7)
    zeroZone = (-1, -1)
    if(lFound and rFound):
        cv2.cornerSubPix(leftRectified[-1], lCorners, winSize, zeroZone,
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1000, 0.001))
        cv2.cornerSubPix(rightRectified[-1], rCorners, winSize, zeroZone,
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1000, 0.001))
        ##draw and save images
        totalImagesFound+=1
        RMSimage = 0
        #######
        ##Draw Corners
        drawnLeft = copy.deepcopy(cv2.cvtColor(leftRectified[-1], cv2.COLOR_GRAY2RGB))
        drawnRight = copy.deepcopy(cv2.cvtColor(rightRectified[-1], cv2.COLOR_GRAY2RGB))
        cv2.drawChessboardCorners(drawnLeft,calibrationData.inCalibrationData.meta.getBoardSize(), lCorners, lFound)
        cv2.drawChessboardCorners(drawnRight,calibrationData.inCalibrationData.meta.getBoardSize(), rCorners, rFound)
        Combined = np.hstack((drawnLeft, drawnRight))
        for i in range(len(lCorners)):
            RMSimage+=np.power((lCorners[i]-rCorners[i])[0,1],2)
            epiPolarError+=np.power((lCorners[i]-rCorners[i])[0,1],2)
            cv2.line(Combined, (lCorners[i][0][0], lCorners[i][0][1]),
                     (int(rCorners[i][0][0] + leftRectified[-1].shape[1]), rCorners[i][0][1]), (0, 255, 0))
        RMSimage=np.sqrt(RMSimage/len(lCorners))
        epiError.append(RMSimage)
        cv2.imwrite(outFolder+"/"+fileName,Combined)
    else:
        print("NO CHECKER")
    outshow=copy.deepcopy(leftRectified[-1])
    cv2.imshow("outWindow",Combined)
    cv2.waitKey(30)

totalPoints=(totalImagesFound*
             calibrationData.inCalibrationData.meta.getBoardSize()[0]*
             calibrationData.inCalibrationData.meta.getBoardSize()[1])
print(totalPoints)
epiPolarError= np.sqrt(epiPolarError/totalPoints)
print(epiPolarError)

f1 = plt.figure()
ax1 = f1.add_subplot(111)
ax1.hist(epiError,25)
ax1.set_xlabel("Average Epipolar Error")
ax1.set_ylabel("Total Images")

print("Baseline")
print(extrin.getBaseline())

print(intrin.lROI)
print(intrin.rROI)
perc=100*(intrin.lROI[2]*intrin.lROI[3]/float(calibrationData.inCalibrationData.meta.imgSize[0]*
                                     calibrationData.inCalibrationData.meta.imgSize[1]))
print("coverage %"+ str(perc))
leftCoverage=copy.deepcopy(calibrationData.inCalibrationData.showImageCoverage()[0])
rightCoverage=copy.deepcopy(calibrationData.inCalibrationData.showImageCoverage()[1])

leftCorner1=(intrin.lROI[1],intrin.lROI[0])##y and x swopped around in np
leftCorner2=(intrin.lROI[1]+intrin.lROI[3],intrin.lROI[0]+intrin.lROI[2])

rightCorner1=(intrin.rROI[1],intrin.rROI[0])
rightCorner2=(intrin.rROI[1]+intrin.rROI[3],intrin.rROI[0]+intrin.rROI[2])

cv2.rectangle(leftCoverage,leftCorner1,leftCorner2,(0,0,255),3)
cv2.rectangle(rightCoverage,rightCorner1,rightCorner2,(0,0,255),3)

Coverage=np.hstack((leftCoverage, rightCoverage))

for i in range(35):
    cv2.line(Coverage, (0, int(i*Coverage.shape[0]/35.0)),
             (Coverage.shape[1], int(i*Coverage.shape[0]/35.0)), (0, 0, 0,0.3))

cv2.line(Coverage, (int(Coverage.shape[1]/2.0),0),
             (int(Coverage.shape[1]/2.0), Coverage.shape[1]), (0, 0, 0,1))

cv2.imshow("coverage",Coverage)

cv2.imwrite(calibrationDirectory+"/Coverage.png",Coverage)
cv2.waitKey(1000)

plt.savefig(calibrationDirectory+"/EpiErrorFigure.png",format='png',dpi=1200)
plt.show()





