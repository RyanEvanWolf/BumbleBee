#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

from bumblebee_calibration import *

import pickle

inputDir=sys.argv[1]
outputDir=sys.argv[2]
squareWidth=float(sys.argv[3])
nWidth=int(sys.argv[4])
nHeight=int(sys.argv[5])
distortModel=sys.argv[6]
debayer=bool(sys.argv[7])

settings=calibrationMetaData(sys.argv)
settings.printSettings()
outputData=SingleCameraCalData()
outputData.meta=settings
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
    if(outputData.meta.debayer):
        fullImage=cv2.cvtColor(fullImage,cv2.COLOR_BAYER_BG2GRAY)
    leftImage=fullImage[fullImage.shape[0]/2:fullImage.shape[0], 0:fullImage.shape[1]]
    outputData.meta.imgSize=leftImage.shape
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
        outputData.LeftPoints.append(lCorners)
        outputData.RightPoints.append(rCorners)
        outputData.PatternPoints.append(objp)
        outputData.imgName.append(imgDir)
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


if(outputData.meta.distortionModel=="4Parameter"):
    ##fix
    print("4Parameter Distortion Model Detected")
    singleCameraCalibrationFlags=cv2.CALIB_FIX_K3
    outputData.Lparam.calibrationFlags = cv2.CALIB_FIX_K3
    outputData.Rparam.calibrationFlags = cv2.CALIB_FIX_K3
    Dl = np.array((1, 4), np.float64)
    Dr = np.array((1, 4), np.float64)
else:
    print("Assuming 5Parameter Distortion Model")
    outputData.meta.distortionModel="5Parameter"
    Dl = np.array((1, 5), np.float64)
    Dr = np.array((1, 5), np.float64)
    singleCameraCalibrationFlags=0
    outputData.Lparam.calibrationFlags=0
    outputData.Rparam.calibrationFlags=0

print("left Camera")

Kl=np.zeros((3,3), np.float64)
R=None
T=None

outputData.Lparam.RMS_error,outputData.Lparam.K,outputData.Lparam.D,outputData.LR,outputData.LT= cv2.calibrateCamera(np.array(outputData.PatternPoints),np.array(outputData.LeftPoints),
                                                                                                                        leftImage.shape[::-1],Kl,Dl,R,T,flags=singleCameraCalibrationFlags,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 600, 0.05))
print("LeftRMS ERROR := ",outputData.Lparam.RMS_error)

totalMeasurements=0
totalError=0
otherError=0
euclidianDistance=[]
for imgIdx in range(len(outputData.LR)):
    print("imgNumber",imgIdx)
    count=0
    imgReprojected=[]
    for PtsIdx in np.array(objp):
        reprojected,jacob=cv2.projectPoints(PtsIdx.reshape((1,3)),
                          np.array(outputData.LR[imgIdx]).reshape(1, 3),
                          np.array(outputData.LT[imgIdx]).reshape(1, 3),
                          outputData.Lparam.K,outputData.Lparam.D)
        imgReprojected.append(reprojected)
        #totalError+=np.square(reprojected-LeftCorners[imgIdx][count]).sum()
        #otherError+=np.linalg.norm(reprojected-LeftCorners[imgIdx][count])**2
        #avgEuclidian.append(np.linalg.norm(reprojected-LeftCorners[imgIdx][count]))
        count=count+1
        totalMeasurements+=1
    outputData.LeftReprojected.append(imgReprojected)

print("Right Camera")
Kr = np.zeros((3, 3), np.float64)
R = None
T = None
outputData.Rparam.RMS_error, outputData.Rparam.K, outputData.Rparam.D, outputData.RR, outputData.RT = cv2.calibrateCamera(
        np.array(outputData.PatternPoints), np.array(outputData.RightPoints),
        rightImage.shape[::-1], Kr, Dr, R, T, flags=singleCameraCalibrationFlags,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 600, 0.05))
print("RightRMS ERROR := ", outputData.Rparam.RMS_error)

totalMeasurements = 0
totalError = 0
otherError = 0
euclidianDistance = []
for imgIdx in range(len(outputData.RR)):
    print("imgNumber", imgIdx)
    count = 0
    imgReprojected = []
    for PtsIdx in np.array(objp):
        reprojected, jacob = cv2.projectPoints(PtsIdx.reshape((1, 3)),
                                                              np.array(outputData.RR[imgIdx]).reshape(1, 3),
                                                              np.array(outputData.RT[imgIdx]).reshape(1, 3),
                                                              outputData.Rparam.K, outputData.Rparam.D)
        imgReprojected.append(reprojected)
        count = count + 1
        totalMeasurements += 1
    outputData.RightReprojected.append(imgReprojected)



# outputData.Rparam.RMS_error,outputData.Rparam.Kcam,outputData.Rparam.D,outputData.RR,outputData.RT= cv2.calibrateCamera(np.array(outputData.PatternPoints),np.array(outputData.rightPoints),
#                                                                                                                         rightImage.shape[::-1],Kl,Dl,R,T,flags=singleCameraCalibrationFlags,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 600, 0.05))
# print("RightRMS ERROR := ",outputData.Rparam.RMS_error)
#
#
# totalMeasurements=0
# totalError=0
# otherError=0
# euclidianDistance=[]
# for imgIdx in range(len(outputData.RR)):
#     print("imgNumber",imgIdx)
#     count=0
#     for PtsIdx in np.array(objp):
#         outputData.RightReprojected,jacob=cv2.projectPoints(PtsIdx.reshape((1,3)),
#                           np.array(outputData.RR[imgIdx]).reshape(1, 3),
#                           np.array(outputData.RT[imgIdx]).reshape(1, 3),
#                           outputData.Rparam.K,outputData.Rparam.D)
#         #totalError+=np.square(reprojected-LeftCorners[imgIdx][count]).sum()
#         #otherError+=np.linalg.norm(reprojected-LeftCorners[imgIdx][count])**2
#         #avgEuclidian.append(np.linalg.norm(reprojected-LeftCorners[imgIdx][count]))
#         count=count+1
#         totalMeasurements+=1



outputFolder=outputData.meta.outDirectory +"/singleCalibrationData.p"
print("Saving to File "+outputFolder)
pickle.dump(outputData,open(outputFolder,"wb"))

# print("calculated",np.sqrt(totalError/totalMeasurements))
# print("calculated2",np.sqrt(otherError/totalMeasurements))
# print(totalMeasurements)
# print(objp.shape)
#
#
print("COMPLETED")
#
# plt.hist(range(euclidianDistance),euclidianDistance)
# plt.savefig("/home/ryan/leftHist.png",dpi=300)
#
# tempFiles = []
# count=0
