#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

import scipy.stats.mstats as stat

import matplotlib.pyplot as plt
from bumblebee_calibration import *


mappingName="_Map.p"
calibrationName="StereoCalibration.p"
singleName="inputStereoCalibration.p"
extrinName="extrinsicCalibration.p"
intrinName="intrinsicCalibration.p"


import pickle


inputData=sys.argv[1]
outputDirectory=sys.argv[2]

outputData=StereoCameraInfo()



outputData.calibrateFromFile(inputData)
print("saving to output Folder "+outputDirectory)

if(not os.path.isdir(outputDirectory)):
        os.makedirs(outputDirectory)


pickle.dump(outputData,open(outputDirectory+"/"+calibrationName,"wb"))
pickle.dump(outputData.extrin,open(outputDirectory+"/"+extrinName,"wb"))
pickle.dump(outputData.intrin,open(outputDirectory+"/"+intrinName,"wb"))
pickle.dump(outputData.inCalibrationData,open(outputDirectory+"/"+singleName,"wb"))
pickle.dump(outputData.lRect,open(outputDirectory+"/left"+mappingName,"wb"))
pickle.dump(outputData.rRect,open(outputDirectory+"/right"+mappingName,"wb"))
print("COMPLETED")

