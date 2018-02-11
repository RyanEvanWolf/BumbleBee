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

inputData=sys.argv[1]+"/Calibration/singleCalibrationData.p"
filteredOut=sys.argv[2]
if(len(sys.argv)>3):
    errorThreshold=float(sys.argv[3])
else:
    errorThreshold=1.5

print("Loading calibration data from "+inputData)
calibrationData=pickle.load(open(inputData, "rb" ))
print("file opened")
plt.style.use('seaborn')



print(calibrationData.getRMSreprojection())

leftImagesAvgRms=[]
rightImagesAvgRms=[]
combined=[]

for i in range(len(calibrationData.LeftPoints)):
    leftImagesAvgRms.append(calibrationData.getSingleImageRMS(i)[0])
    rightImagesAvgRms.append(calibrationData.getSingleImageRMS(i)[1])
    combined.append((calibrationData.getSingleImageRMS(i)[0]+calibrationData.getSingleImageRMS(i)[1])/2.0)


filter=[]
inputIndexes=[]



for i in range(len(calibrationData.LeftPoints)):
    if(combined[i]<errorThreshold):
        filter.append(combined[i])
        inputIndexes.append(i)

f1 = plt.figure()
ax1 = f1.add_subplot(121)
ax1.hist(combined,40)
ax1.set_title("Initial Average RMS Error")
ax1.set_xlabel("Average RMS Error")
ax1.set_ylabel("Total Images")
ax2 = f1.add_subplot(122)
ax2.hist(filter,15)
ax2.set_title("Filtered Average RMS Error")
ax2.set_xlabel("Average RMS Error")
ax2.set_ylabel("Total Images")


if(not os.path.isdir(filteredOut)):
        os.makedirs(filteredOut)
for i in inputIndexes:
    inImageDir=calibrationData.meta.inDirectory+"/"+calibrationData.imgName[i]
    outImageDir=filteredOut+"/"+calibrationData.imgName[i]
    calImage=cv2.imread(inImageDir,cv2.IMREAD_GRAYSCALE)
    print(inImageDir,outImageDir)
    cv2.imwrite(outImageDir,calImage)

print("total thresholded input images := "+str(len(inputIndexes)))
for i in inputIndexes:
    matchedImagedDir=calibrationData.meta.outDirectory+"/Match/"+calibrationData.imgName[i]
    cv2.imshow("a",cv2.imread(matchedImagedDir,cv2.IMREAD_COLOR))
    cv2.waitKey(20)


plt.savefig(sys.argv[1]+"/Calibration/ErrorFigure.png",format='png',dpi=1200)#,figsize=(1000,1000))
plt.show()