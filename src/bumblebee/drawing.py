import numpy as np
import cv2
import copy

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm
import random
from bumblebee.calibration import getROIoverlap

def drawEpiPolarLines(inImage,percentage=0.1):
    nLines=int(inImage.shape[0]*percentage)
    for i in range(nLines):
        cv2.line(inImage, (0, int(i*inImage.shape[0]/nLines)),
                 (inImage.shape[1], int(i*inImage.shape[0]/nLines)), (0, 0, 0,0.3))


def drawCalibrationCoverageMap(rectImages,foundPoints):
    outputImage=255*np.ones(rectImages[0].shape,dtype=np.uint8)
    print(outputImage.shape)
    for checkerBoardIndex in foundPoints:
        for PointIndex in checkerBoardIndex:
            cv2.circle(outputImage,(int(PointIndex[0,0]),int(PointIndex[0,1])),3,(80,20,0,0.1),-1)###column , row
        for PointIndex in range(0,len(checkerBoardIndex)-1):
            pt1=(int(checkerBoardIndex[PointIndex+1][0,0]),int(checkerBoardIndex[PointIndex+1][0,1]))
            pt2=(int(checkerBoardIndex[PointIndex][0,0]),int(checkerBoardIndex[PointIndex][0,1]))
            cv2.line(outputImage,pt1,pt2,(20,80,0,0.1))
    return outputImage

def drawStereoROI(epiImage,leftROI,rightROI):
    offsetX=int(epiImage.shape[1]/2)
    print(offsetX)
    lCol=(0,0,255)
    rCol=lCol
    leftCorner1=(leftROI[1],leftROI[0])#,lCol,3)
    leftCorner2=(leftROI[1]+leftROI[3],leftROI[0]+leftROI[2])
    print(leftCorner1)
    rightCorner1=(rightROI[1]+offsetX,rightROI[0])#+offsetX)#,lCol,3)
    rightCorner2=(rightROI[1]+rightROI[3]+offsetX,rightROI[0]+rightROI[2])#+offsetX)
    print(rightCorner1)
    print(rightCorner2)
    cv2.rectangle(epiImage,leftCorner1,leftCorner2,
                lCol,3)
    cv2.rectangle(epiImage,rightCorner1,rightCorner2,
                rCol,3)
    overlapROI=getROIoverlap(leftROI,rightROI)

    overlapImage=copy.deepcopy(epiImage)
    cv2.rectangle(overlapImage, (overlapROI[1],overlapROI[0]),
                   (overlapROI[1]+overlapROI[3],overlapROI[0]+overlapROI[2]),(0,255,0),-1)
    cv2.rectangle(overlapImage, (overlapROI[1]+offsetX,overlapROI[0]),
                   (overlapROI[1]+overlapROI[3]+offsetX,overlapROI[0]+overlapROI[2]),(0,255,0),-1)
    epiImage=copy.deepcopy(cv2.addWeighted(epiImage, 0.7, overlapImage, 0.2, 0))
    return epiImage

def plotErrorHist(rmsError,xlabel,ylabel="Total Images",bins=25,displayPDF=True):
    fig, ax1 = plt.subplots()
    y, x, _ =ax1.hist(rmsError,bins)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    if(displayPDF):
        ax2=ax1.twinx()
        mu,sigma = norm.fit(rmsError)
        x_pdf = np.linspace(0,np.max(rmsError), 100)
        y_pdf=norm.pdf(x_pdf, mu, sigma)
        ax2.plot(x_pdf,y_pdf,color='r',linestyle='dashed')
        ax2.set_ylabel('Probability Density Function')
    return fig,ax1

# s2 = np.sin(2 * np.pi * t)
# ax2.plot(t, s2, 'r.')
# ax2.set_ylabel('sin', color='r')
# ax2.tick_params('y', colors='r')

# fig.tight_layout()
# plt.show()

# def plotEpiError(epiError):
#     f1 = plt.figure()
#     ax1 = f1.add_subplot(111)
#     ax1.hist(rmsError,25)
#     ax1.set_xlabel("Average Epipolar Error")
#     ax1.set_ylabel("Total Images")
#     (mu,sigma) = norm.fit(rmsError)

#     return f1,ax1    
