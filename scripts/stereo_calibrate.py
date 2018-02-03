#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

import matplotlib.pyplot as plt
from bumblebee_calibration import *

import pickle

inputData=sys.argv[1]
errorThreshold=1.0

print("Loading calibration data from "+inputData)
calibrationData=pickle.load(open(inputData, "rb" ))
print("file opened")

print(calibrationData.getRMSreprojection())

leftImagesAvgRms=[]
rightImagesAvgRms=[]

for i in range(len(calibrationData.LeftPoints)):
    leftImagesAvgRms.append(calibrationData.getSingleImageRMS(i)[0])
    rightImagesAvgRms.append(calibrationData.getSingleImageRMS(i)[1])

leftFilter=[]
rightFilter=[]

filterl=[]
filterr=[]

for i in range(len(calibrationData.LeftPoints)):
    if((calibrationData.getSingleImageRMS(i)[0]<errorThreshold)and(calibrationData.getSingleImageRMS(i)[1]<errorThreshold)):
        leftFilter.append([i,calibrationData.getSingleImageRMS(i)[0]])
        filterl.append(calibrationData.getSingleImageRMS(i)[0])
        rightFilter.append([i,calibrationData.getSingleImageRMS(i)[1]])
        filterr.append(calibrationData.getSingleImageRMS(i)[1])

f1 = plt.figure()
f2 = plt.figure()
ax1 = f1.add_subplot(121)
ax1.hist(leftImagesAvgRms,30)
ax1.set_title("Left Images RMS")
ax2 = f1.add_subplot(122)
ax2.hist(rightImagesAvgRms,30)
ax2.set_title("Right Images RMS")

ax3 = f2.add_subplot(121)
ax3.hist(filterl,20)
ax3.set_title("Filtered Left Images RMS")
ax4 = f2.add_subplot(122)
ax4.hist(filterr,20)
ax4.set_title("Filtered Right Images RMS")
plt.show()
