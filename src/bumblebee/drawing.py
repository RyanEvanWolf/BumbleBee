import numpy as np
import cv2
import copy

import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from scipy.stats import norm
import random
def drawEpiPolarLines(inImage,percentage=0.1):
    pass


def drawCalibrationCoverageMap(rectImages,foundPoints,roi,roiOverlap):
    outputImage=200*np.ones(rectImages[0].shape,dtype=np.uint8)
    print(outputImage.shape)
    for checkerBoardIndex in foundPoints:
        for PointIndex in checkerBoardIndex:
            cv2.circle(outputImage,(int(PointIndex[0,0]),int(PointIndex[0,1])),3,(0,0,50,0.1),-1)###column , row
        for PointIndex in range(0,len(checkerBoardIndex)-1):
            pt1=(int(checkerBoardIndex[PointIndex+1][0,0]),int(checkerBoardIndex[PointIndex+1][0,1]))
            pt2=(int(checkerBoardIndex[PointIndex][0,0]),int(checkerBoardIndex[PointIndex][0,1]))
            cv2.line(outputImage,pt1,pt2,(0,0,50,0.1))
    for i in random.sample(range(0,len(foundPoints)),5):
        cv2.drawChessboardCorners(outputImage,(11,8),foundPoints[i],True)
    return outputImage


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
