# BumbleBee
A ros based package that performs stereo processing for the depracated BumbleBee camera from PointGrey

-----------
How to run the calibration Scripts
----------



calibrate.py
----
Generates all the calibration data for a stereo bumblebee image pair which includes the actual 
calibrated values K and D...together with the reprojected points and checkerboard transforms R,T.
These are then filtered according to their RMS error in each image by running filterCalibration.py
StereoCalibration is then performed

1)



roslaunch bumblebee calFromFile.xml inDir:="/home/ryan/currentTesting/8e" debayer:="True" outDir:="/home/ryan/currentTesting/8eInitial" checkerBoardWidth:="12" checkerBoardHeight:="9" squareLength:="0.045"
2)



rosrun bumblebee filterCalibration.py /home/ryan/currentTesting/8eInitial/Calibration/singleCalibrationData.p /home/ryan/currentTesting/8eFiltered
3)
Rerun 1) with the newfiltered set of images 



roslaunch bumblebee calFromFile.xml inDir:="/home/ryan/currentTesting/8eFiltered" debayer:="True" outDir:="/home/ryan/currentTesting/stereoIn" checkerBoardWidth:="12" checkerBoardHeight:="9" squareLength:="0.045"
4)
StereoCalibrate with the filtered calibration data


 rosrun bumblebee stereo_calibrate.py /home/ryan/currentTesting/stereoIn/Calibration/singleCalibrationData.p /home/ryan/currentTesting/stereoOut
 
5)
Analyse the stereo calibration results

rosrun bumblebee analyseCalibration.py /home/ryan/currentTesting/stereoOut


 


