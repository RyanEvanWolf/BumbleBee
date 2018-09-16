# BumbleBee
A ros based package that performs stereo processing for the depracated BumbleBee camera from PointGrey

-----------
How to run the calibration Scripts
----------



calibrate.py
----
Generates all the calibration data for a stereo bumblebee image pair which includes the actual 
calibrated values K and D...together with the reprojected points and checkerboard transforms R,T.
These are then filtered according to their RMS error in each image by running filterCalibration
StereoCalibration is then performed with the calibrationData being used as the initial guess into the system.

1)

----------
---performs initial calibration on a set of stereo images recorded from the libdc1394 images.
---if no output folder is specified, it creates a singleCalibrationData.p pickled object with all the information acquired for both the left and right cameras.
---checkerboard patterns are drawn and saved in the relevant directories indicating whether it was found or not.
rosrun bumblebee opencvCalibration --inputDir AbsoluteFolderToImages

example: rosrun bumblebee opencvCalibration /home/ryan/CalibrationInput/OriginalImages
2)
-----------------
---because we do not know which are good and bad calibration images, we first filter out image pairs which have a poor reprojection error.
---a new folder is then created where all calibration images are stored if their RMS reprojection error is below a definable threshold level


example: rosrun bumblebee filterCalibration /home/ryan/CalibrationInput/singleCalibrationData.p ~/filteredOutputB/filteredImages --threshold 1.2

3)
------------
--Rerun 1) with the newfiltered set of images 
--
example:rosrun bumblebee opencvCalibration /home/ryan/filteredOutputB/filteredImages

4)
------------
--Given a singleCalibrationData.p file, calibrate a stereo pair based on the initial camera configuration
--- Note: the input argument is a folder, not the absolute file

example: rosrun bumblebee opencvStereoCalibration /home/ryan/filteredOutputA/singleCalibrationData.p /home/ryan/CalibrationA

 rosrun bumblebee stereo_calibrate.py /home/ryan/currentTesting/stereoIn/Calibration/singleCalibrationData.p /home/ryan/currentTesting/stereoOut
 
5)
Analyse the stereo calibration results

rosrun bumblebee analyseCalibration.py /home/ryan/currentTesting/stereoOut


 


