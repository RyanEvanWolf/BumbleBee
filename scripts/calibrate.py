#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

import matplotlib.pyplot as plt
from bumblebee_calibration import *

inputDir=sys.argv[1]
outputDir=sys.argv[2]
squareWidth=float(sys.argv[3])
nWidth=int(sys.argv[4])
nHeight=int(sys.argv[5])
distortModel=sys.argv[6]
debayer=bool(sys.argv[7])


print("Settings: ")
print("inputDir",inputDir)
print("outputDir",outputDir)
print("squareWidth",squareWidth)
print("nWidth",nWidth)
print("nHeight",nHeight)
print("distortModel",distortModel)
print("debayer",debayer)

##########
##Create output Directory Structure
###########

FoundDir=outputDir+"/Found"
MatchDir=outputDir+"/Match"
RectDir=outputDir+"/Rectified"
NotFoundDir=outputDir+"/NotFound"

MaskedImgs=[]##holds all the images which were not found in previous runs

folders=[FoundDir,MatchDir,RectDir]

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros(((nWidth-1)*(nHeight-1),3), np.float32)
objp[:,:2] = squareWidth*np.mgrid[0:(nWidth-1),0:(nHeight-1)].T.reshape(-1,2)

def generateBoardCoordinates():
    ###create a single chessboard coordinates assuming 1 point starts at 0
    outputCoordinates=[]
    for w in range(0,nWidth-1):
        for h in range(0,nHeight-1):
            boardCoordinate=[w*squareWidth,h*squareWidth,0]
            outputCoordinates.append(boardCoordinate)
    return outputCoordinates

for i in folders:
    if(not os.path.isdir(i)):
        os.makedirs(i)

if(not os.path.isdir(NotFoundDir)):
    os.makedirs(NotFoundDir)
else:
    for files in os.listdir(NotFoundDir):
        MaskedImgs.append(files)
    print("TOTAL FILES REMOVED FROM OPTIMIZATION = "+str(len(MaskedImgs)))
##################
###Get all the image Dir
###############

inImageFiles=[]
for files in os.listdir(inputDir):
    if files not in MaskedImgs:
        inImageFiles.append(files)
    else:
        print("image Masked : "+files)

print("Searching for Corners")
#
# print(np.float32([[3,0,0]]).reshape(-1,3))
# print(np.array(((800,0,500),(0,800,450),(0,0,1))))
# print(cv2.projectPoints(np.float32([[3,0,0]]).reshape(-1,3),
#                         ((0,0,0)),
#                         ((0,0,0)),
#                         np.array(((800,0,500),(0,800,450),(0,0,1)),np.float32),
#                         ((0.01,0.02,0.03,0.04,0.05))))




countCycle=0


viewImage=cv2.namedWindow("Found",cv2.WINDOW_NORMAL)

winSize=(7,7)
zeroZone=(-1,-1)

LeftCorners=[]
RightCorners=[]
patternPoints=[]

for imgDir in inImageFiles:
    print("["+str(countCycle)+"/"+str(len(inImageFiles))+"]")
    fullImageDir = inputDir + "/" + imgDir
    fullImage = cv2.imread(fullImageDir, cv2.IMREAD_GRAYSCALE)
    if(debayer):
        fullImage=cv2.cvtColor(fullImage,cv2.COLOR_BAYER_BG2GRAY)
    leftImage=fullImage[fullImage.shape[0]/2:fullImage.shape[0], 0:fullImage.shape[1]]
    rightImage = fullImage[0:fullImage.shape[0] / 2, 0:fullImage.shape[1]]
    lFound,lCorners=cv2.findChessboardCorners(leftImage,(nWidth-1,nHeight-1),cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE)
    rFound,rCorners=cv2.findChessboardCorners(rightImage,(nWidth-1,nHeight-1),cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE)
    if(lFound and rFound):
        cv2.cornerSubPix(leftImage, lCorners, winSize, zeroZone, (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1000, 0.001))
        cv2.cornerSubPix(rightImage, rCorners, winSize, zeroZone,
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 1000, 0.001))
        #######
        ##Draw Corners
        drawnLeft=cv2.cvtColor(leftImage,cv2.COLOR_GRAY2RGB)
        drawnRight=cv2.cvtColor(rightImage,cv2.COLOR_GRAY2RGB)
        cv2.drawChessboardCorners(drawnLeft, (nWidth-1,nHeight-1), lCorners, lFound)
        cv2.drawChessboardCorners(drawnRight, (nWidth - 1, nHeight - 1), rCorners, rFound)

        cv2.imwrite(FoundDir+"/left_"+imgDir,drawnLeft)
        cv2.imwrite(FoundDir + "/right_" + imgDir, drawnRight)
        #######
        ##Draw Matches
        Combined=np.hstack((drawnLeft,drawnRight))
        for i in range(len(lCorners)):
            cv2.line(Combined,(lCorners[i][0][0],lCorners[i][0][1]),(int(rCorners[i][0][0]+leftImage.shape[1]),rCorners[i][0][1]),(0,255,0))
        cv2.imwrite(MatchDir+"/"+imgDir,Combined)
        LeftCorners.append(lCorners)
        RightCorners.append(rCorners)
        patternPoints.append(objp)
        cv2.imshow("Found",drawnLeft)
        cv2.waitKey(500)
    else:
        ####
        ##save images Not Found

        cv2.imwrite(NotFoundDir+"/"+imgDir,cv2.imread(fullImageDir, cv2.IMREAD_GRAYSCALE))
        print("no ChessBoard Found")
    countCycle=countCycle+1

cv2.destroyAllWindows()
print("Beginning Single Camera Calibrations")


if(distortModel=="4Parameter"):
    ##fix
    print("4Parameter Distortion Model Detected")
    singleCameraCalibrationFlags=cv2.CALIB_FIX_K3
    Dl = np.array((1, 4), np.float64)
    Dr = np.array((1, 4), np.float64)
else:
    print("Assuming 5Parameter Distortion Model")
    Dl = np.array((1, 5), np.float64)
    Dr = np.array((1, 5), np.float64)
    singleCameraCalibrationFlags=0


print("left Camera")
Kl=np.zeros((3,3), np.float64)
R=None
T=None

rms,cam,D,rvecs,tvecs= cv2.calibrateCamera(np.array(patternPoints),np.array(LeftCorners), leftImage.shape[::-1],Kl,Dl,R,T,flags=singleCameraCalibrationFlags,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 600, 0.05))

print("LeftRMS ERROR := ",rms)
print(cam)

totalMeasurements=0
totalError=0
otherError=0
euclidianDistance=[]
for imgIdx in range(len(rvecs)):
    print("imgNumber",imgIdx)
    count=0
    avgEuclidian=[]
    for PtsIdx in np.array(objp):
        reprojected,jacob=cv2.projectPoints(PtsIdx.reshape((1,3)),
                          np.array(rvecs[imgIdx]).reshape(1, 3),
                          np.array(tvecs[imgIdx]).reshape(1, 3),
                          cam,D)
        totalError+=np.square(reprojected-LeftCorners[imgIdx][count]).sum()
        otherError+=np.linalg.norm(reprojected-LeftCorners[imgIdx][count])**2
        avgEuclidian.append(np.linalg.norm(reprojected-LeftCorners[imgIdx][count]))
        #print("euclidian distance",np.linalg.norm(reprojected-LeftCorners[imgIdx][count]))
        #euclidianDistance=np.linalg.norm(reprojected-LeftCorners[imgIdx][count])
        #print(RMS+np.sum(np.linalg.norm(reprojected-LeftCorners[imgIdx][count])))
        #RMS =RMS+np.sum(np.linalg.norm(reprojected-LeftCorners[imgIdx][count]))
        count=count+1
        totalMeasurements+=1
        if(count>=154):
            break
    euclidianDistance.append(avgEuclidian)
print("calculated",np.sqrt(totalError/totalMeasurements))
print("calculated2",np.sqrt(otherError/totalMeasurements))
print(totalMeasurements)
print(objp.shape)
        #cv2.projectPoints(np.array(PtsIdx),np.array(rvecs[imgIdx]))
#print(len(patternPoints))
#print(len(LeftCorners))

#print(len(cv2.projectPoints(np.array(patternPoints,np.float64),np.array(rvecs,np.float64),np.array(tvecs,np.float64),np.array(cam,np.float64),np.array(D,np.float64))))
#a=cv2.calibrateCamera(combinedBoardCoordinates,LeftCorners,leftImage.size,Kl,D,R,T,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 150, 0.05))
#print(rms)
#print(camera_matrix)

print("COMPLETED")

plt.boxplot(euclidianDistance)
plt.savefig("/home/ryan/box.png",dpi=300)

tempFiles = []
count=0
# if(not os.path.isdir(outputFile+"/left")):
#     os.makedirs(outputFile+"/left")
# if(not os.path.isdir(outputFile+"/right")):
#     os.makedirs(outputFile+"/right")
#
# for files in os.listdir(inputFile):
#     print(count,len(os.listdir(inputFile)))
#     inImage=cv2.imread(inputFile+"/"+files,cv2.IMREAD_GRAYSCALE)
#     rImage=inImage[0:inImage.shape[0]/2,0:inImage.shape[1]]
#     lImage = inImage[inImage.shape[0]/2:inImage.shape[0], 0:inImage.shape[1]]
#     #rImage=inImage[0:inImage.shape[1],inImage.shape[2]/2:inImage.shape[1],inImage.shape[2]]
#     cv2.imshow("right",lImage)
#     cv2.waitKey(1)
#     cv2.imwrite(outputFile+"/left/"+str(count).zfill(3)+".PNG",lImage)
#     cv2.imwrite(outputFile + "/right/" + str(count).zfill(3)+".PNG",rImage)
#     count=count+1
#     #tempFiles.append(self.rootDir + "/" + files)