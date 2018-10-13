import cv2
import numpy as np
##create a high res calibration image for A2 paper

height=9
width=12
borderPixels=150

imageWidth=4676
imageHeight=3306
squareSize1=int((imageWidth-2*borderPixels)/width)
squareSize2=int((imageHeight-2*borderPixels)/height)
if(squareSize1<squareSize2):
    squareSize=squareSize1
else:
    squareSize=squareSize2
print(squareSize)
overallImage=255*np.ones((imageHeight,imageWidth,3),np.int16)

for h in range(height):
    if(bool(h%2)):
        firstColour=(255,255,255)
        secondColour=(0,0,0)
        print(h)
        ##start with black
    else:
        firstColour=(0,0,0)
        secondColour=(255,255,255)
        print(h)
        ##start with white
    for w in range(width):
        if(w%2):
            cv2.rectangle(overallImage,(borderPixels+w*squareSize,borderPixels+h*squareSize),
                          (borderPixels+w*squareSize+squareSize,squareSize+borderPixels+h*squareSize),firstColour,-1)
        else:
            cv2.rectangle(overallImage, (borderPixels + w * squareSize, borderPixels + h * squareSize),
                          (borderPixels + w * squareSize + squareSize, squareSize + borderPixels + h * squareSize),secondColour, -1)
cv2.imwrite("checkerBoard.ppm",overallImage)
