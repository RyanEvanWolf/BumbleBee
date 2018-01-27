#!/usr/bin/env python

import rospy

import time

import cv2

import sys
import os

inputFile=sys.argv[1]
outputFile=sys.argv[2]


print(inputFile)
print(outputFile)

tempFiles = []
count=0
if(not os.path.isdir(outputFile+"/left")):
    os.makedirs(outputFile+"/left")
if(not os.path.isdir(outputFile+"/right")):
    os.makedirs(outputFile+"/right")

for files in os.listdir(inputFile):
    print(count,len(os.listdir(inputFile)))
    inImage=cv2.imread(inputFile+"/"+files,cv2.IMREAD_GRAYSCALE)
    rImage=inImage[0:inImage.shape[0]/2,0:inImage.shape[1]]
    lImage = inImage[inImage.shape[0]/2:inImage.shape[0], 0:inImage.shape[1]]
    #rImage=inImage[0:inImage.shape[1],inImage.shape[2]/2:inImage.shape[1],inImage.shape[2]]
    cv2.imshow("right",lImage)
    cv2.waitKey(1)
    cv2.imwrite(outputFile+"/left/"+str(count).zfill(3)+".PNG",lImage)
    cv2.imwrite(outputFile + "/right/" + str(count).zfill(3)+".PNG",rImage)
    count=count+1
    #tempFiles.append(self.rootDir + "/" + files)
