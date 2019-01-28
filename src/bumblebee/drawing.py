import numpy as np
import cv2
import copy

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm
import random
from bumblebee.calibration import getROIoverlap
from bumblebee.baseTypes import *
from bumblebee.utils import *

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


def drawROI(leftROI,rightROI,overlapROI,imgSize=(1024,768)):
    cameraImage=np.zeros((imgSize[1],imgSize[0],3), np.int8)
    overlapImage=np.zeros((imgSize[1],imgSize[0],3), np.int8)
    print((leftROI[1],leftROI[0]),(leftROI[1]+leftROI[3],leftROI[0]+leftROI[2]))
    cv2.rectangle(cameraImage, (leftROI[0],leftROI[1]),
                   (leftROI[0]+leftROI[2],leftROI[1]+leftROI[3]),(0,0,255),10)
    cv2.rectangle(cameraImage, (rightROI[0],rightROI[1]),
                   (rightROI[0]+rightROI[2],rightROI[1]+rightROI[3]),(255,0,0),10)
    cv2.rectangle(overlapImage, (overlapROI[0],overlapROI[1]),
                   (overlapROI[0]+overlapROI[2],overlapROI[1]+overlapROI[3]),(0,255,0),-1)
    overallImage=cv2.addWeighted(cameraImage, 0.7, overlapImage, 0.2, 0)


    cv2.imwrite("/home/ryan/CALIMAGE.ppm",overallImage)
    return overallImage

def drawROIbox(inImage,roi,col=(0,0,255),drawOn=False):
    if(drawOn):
        cv2.rectangle(inImage,(roi[0],roi[1]),
                    (roi[0]+roi[2],roi[1]+roi[3]),
                    col,2)  
    else:
        outImage=copy.deepcopy(inImage)
        cv2.rectangle(outImage,(roi[0],roi[1]),
                    (roi[0]+roi[2],roi[1]+roi[3]),
                    col,2)         
        return outImage
def drawROIarea(inImage,roi,col=(0,0,255),drawOn=False):

    boxImage=copy.deepcopy(inImage)
    if(drawOn):
        cv2.rectangle(boxImage,(roi[0],roi[1]),
                    (roi[0]+roi[2],roi[1]+roi[3]),
                    col,-1)  
        inImage=copy.deepcopy(cv2.addWeighted(inImage, 0.7, boxImage, 0.2, 0))
    else:
        cv2.rectangle(boxImage,(roi[0],roi[1]),
                    (roi[0]+roi[2],roi[1]+roi[3]),
                    col,-1)         
        return copy.deepcopy(cv2.addWeighted(inImage, 0.7, boxImage, 0.2, 0))
def drawROI(inImage,roi):
    lCol=(0,0,255)
    leftCorner1=(roi[0],roi[1])#,lCol,3) 1,0
    leftCorner2=(roi[0]+roi[2],roi[1]+roi[3])    #1,3,0,2
    cv2.rectangle(inImage,leftCorner1,leftCorner2,
                lCol,3)
def drawStereoROI(epiImage,leftROI,rightROI):
    offsetX=int(epiImage.shape[1]/2)
    lCol=(0,0,255)
    rCol=lCol
    leftCorner1=(leftROI[0],leftROI[1])#,lCol,3) 1,0
    leftCorner2=(leftROI[0]+leftROI[2],leftROI[1]+leftROI[3])    #1,3,0,2

    rightCorner1=(rightROI[0]+offsetX,rightROI[1])#+offsetX)#,lCol,3)    1 0
    rightCorner2=(rightROI[0]+rightROI[2]+offsetX,rightROI[1]+rightROI[3])#+offsetX)   1 3 0 2

    cv2.rectangle(epiImage,leftCorner1,leftCorner2,
                lCol,3)
    cv2.rectangle(epiImage,rightCorner1,rightCorner2,
                rCol,3)
    overlapROI=getROIoverlap(leftROI,rightROI)

    overlapImage=copy.deepcopy(epiImage)
    cv2.rectangle(overlapImage, (overlapROI[0],overlapROI[1]),
                   (overlapROI[0]+overlapROI[2],overlapROI[1]+overlapROI[3]),(0,255,0),-1)
    cv2.rectangle(overlapImage, (overlapROI[0]+offsetX,overlapROI[1]),
                   (overlapROI[0]+overlapROI[2]+offsetX,overlapROI[1]+overlapROI[3]),(0,255,0),-1)
    epiImage=copy.deepcopy(cv2.addWeighted(epiImage, 0.7, overlapImage, 0.2, 0))
    return copy.deepcopy(epiImage)

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

def multipleErrorHist(rmsErrorList,colourList,keylist,xlabel,ylabel="Total Images",bins=25):
    fig, ax1 = plt.subplots()
    ax2=ax1.twinx()
    ax2.grid(False)
    ax1.set_xlabel(xlabel,fontsize=18)
    ax1.set_ylabel(ylabel,fontsize=18)
    ax2.set_ylabel('Probability Density Function',fontsize=18)
    for histIndex in range(0,len(rmsErrorList)):
        ax1.hist(rmsErrorList[histIndex],bins,edgecolor=(0,0,0),color=colourList[histIndex],label=keylist[histIndex],alpha=0.3)
        mu,sigma = norm.fit(rmsErrorList[histIndex])
        x_pdf = np.linspace(0,np.max(rmsErrorList[histIndex]), 100)
        y_pdf=norm.pdf(x_pdf, mu, sigma)
        ax2.plot(x_pdf,y_pdf,color=colourList[histIndex],linestyle='dashed')
    ax1.legend(loc="upper right",fontsize=20)

    fig.savefig("/home/ryan/"+xlabel+".png",format='png',bbox_inches='tight')
    return fig,ax1,ax2
#     fig, ax1 = plt.subplots()
# y, x, _ =ax1.hist(rmsError,bins)
# ax1.set_xlabel(xlabel)
# ax1.set_ylabel(ylabel)
# if(displayPDF):
#     ax2=ax1.twinx()
#     mu,sigma = norm.fit(rmsError)
#     x_pdf = np.linspace(0,np.max(rmsError), 100)
#     y_pdf=norm.pdf(x_pdf, mu, sigma)
#     ax2.plot(x_pdf,y_pdf,color='r',linestyle='dashed')
#     ax2.set_ylabel('Probability Density Function')
def plotTracks(inImage,Track1,Track2,col1=(0,255,0),col2=(0,0,255)):
    for i in range(0,len(Track1)):
        cv2.line(inImage,np2cvDisplay(Track1[i]),
                        np2cvDisplay(Track2[i]),(0,0,0))
        cv2.circle(inImage,np2cvDisplay(Track1[i]),2,col1,-1)    
        cv2.circle(inImage,np2cvDisplay(Track2[i]),2,col2,-1) 
        #cv2.circle(inImage,np2cvDisplay(Track2[i]),col2,-1) 

def plotEpiPolar(inImage,leftTracks,rightTracks,col1=(100,100,255),col2=(255,0,255)):
    offset=inImage.shape[1]/2
    cv2.line(inImage,(offset,0),(offset,inImage.shape[0]),(0,0,0))
    for trackN in range(0,len(leftTracks)):
        cv2.circle(inImage,np2cvDisplay(leftTracks[trackN]),2,col1,-1)
        withoffset=np2cvDisplay(rightTracks[trackN])
        withoffset=(withoffset[0]+offset,withoffset[1])
        cv2.circle(inImage,withoffset,2,col2,-1)
        cv2.line(inImage,np2cvDisplay(leftTracks[trackN]),withoffset,(0,0,0))
        # line=np2cvDisplay(ltrack)
        # line=(offset,line[1])
    #     # cv2.line(inImage,np2cvDisplay(ltrack),line,(0,0,0))
    # for rtrack in rightTracks:
        
    #     withoffset=np2cvDisplay(rtrack)
    #     withoffset=(withoffset[0]+offset,withoffset[1])
    #     line=(offset,withoffset[1])
    #     cv2.circle(inImage,withoffset,2,col2,-1)
    #     cv2.line(inImage,withoffset,line,(0,0,0))
# def plotInterFrameTracks(inImage,interFrame):
#     for tracks in setTrackEdges:
#         Lines=[]
#         for match in tracks.tracks:
#             cv2.circle(inImage,np2cvDisplay(match.L),1,(0,20,255,0.1),-1)
#             Lines.append(np2cvDisplay(match.L))
#         for j in range(1,len(Lines)):
#             cv2.line(inImage,Lines[j-1],Lines[j],(20,80,0,0.1))      
                

# def drawTracks(inImage,setTrackEdges):
#     for tracks in setTrackEdges:
#         Lines=[]
#         for match in tracks.tracks:
#             cv2.circle(inImage,np2cvDisplay(match.L),1,(0,20,255,0.1),-1)
#             Lines.append(np2cvDisplay(match.L))
#         for j in range(1,len(Lines)):
#             cv2.line(inImage,Lines[j-1],Lines[j],(20,80,0,0.1))      
#             #cv2.circle(inImage,np2cvDisplay(match.L),1,(255,20,0,0.1),-1)
        #       cv2.circle(outputImage,(int(PointIndex[0,0]),int(PointIndex[0,1])),3,(80,20,0,0.1),-1)###column , row
        # for PointIndex in range(0,len(checkerBoardIndex)-1):
        #     pt1=(int(checkerBoardIndex[PointIndex+1][0,0]),int(checkerBoardIndex[PointIndex+1][0,1]))
        #     pt2=(int(checkerBoardIndex[PointIndex][0,0]),int(checkerBoardIndex[PointIndex][0,1]))
        #     cv2.line(outputImage,pt1,pt2,(20,80,0,0.1))      
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
