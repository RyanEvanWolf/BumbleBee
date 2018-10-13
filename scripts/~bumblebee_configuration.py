#!/usr/bin/env python
import rospy
from bumblebee_calibration import *

import pickle

from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
from sensor_msgs.msg import CameraInfo,RegionOfInterest,Image
import tf.transformations
########Bumblebee

from cv_bridge import CvBridge


import copy


def msgFromTransform(inMatrix):
    outTransform=Transform()
    T=inMatrix[0:3,3]
    q=tf.transformations.quaternion_from_matrix(inMatrix)
    outTransform.translation.x=T[0]
    outTransform.translation.y=T[1]
    outTransform.translation.z=T[2]
    outTransform.rotation.x=q[0]
    outTransform.rotation.y=q[1]
    outTransform.rotation.z=q[2]
    outTransform.rotation.w=q[3]
    return outTransform

if __name__ == "__main__":
    calibrationDirectory = sys.argv[1]
    nodeName="bumblebee_configuration"
    publishData=False
    if(len(sys.argv)>2):
        nodeName=sys.argv[2]
        publishData=(sys.argv[3]=="True")
    rospy.init_node(nodeName)
    cvb = CvBridge()
    print("Load from "+calibrationDirectory)
    lMap = pickle.load(open(calibrationDirectory + "/left_Map.p", "rb"))
    print("Left Mapping Loaded")
    rMap = pickle.load(open(calibrationDirectory + "/right_Map.p", "rb"))
    print("Right Mapping Loaded")
    extrin = pickle.load(open(calibrationDirectory + "/extrinsicCalibration.p", "rb"))
    print("Camera Extrinsics Loaded")
    intrin = pickle.load(open(calibrationDirectory + "/intrinsicCalibration.p", "rb"))
    print("Camera Intrinsics Loaded")
    stereoCalibration=pickle.load(open(calibrationDirectory + "/StereoCalibration.p", "rb"))
    #############################
    ###create transforms
    transformNames=["world","left","right","left_rectified","right_rectified"]

    chain=extrin.getTransformChain()
    chainTransforms=[]
    for i in range(4):
        chainTransforms.append(TransformStamped())
    #####left Camera
    chainTransforms[0].header.frame_id=transformNames[0]
    chainTransforms[0].child_frame_id=transformNames[1]
    chainTransforms[0].transform=msgFromTransform(np.linalg.inv(chain[0]))
    ####right Camera
    chainTransforms[1].header.frame_id=transformNames[1]
    chainTransforms[1].child_frame_id=transformNames[2]
    chainTransforms[1].transform=msgFromTransform(np.linalg.inv(chain[1]))
    ####left Rectified
    chainTransforms[2].header.frame_id=transformNames[1]
    chainTransforms[2].child_frame_id=transformNames[3]
    chainTransforms[2].transform=msgFromTransform(np.linalg.inv(chain[2]))
    #######right Rectified
    chainTransforms[3].header.frame_id=transformNames[2]
    chainTransforms[3].child_frame_id=transformNames[4]
    chainTransforms[3].transform=msgFromTransform(np.linalg.inv(chain[3]))
    ###############
    ###transforms are inverted because they are coordinate frame to coordinate frame transformations
    ###OpenCV outputs must then be specified in terms of coordinate transformations?
    ####create the publishers
    pub=tf.TransformBroadcaster()
    ####Create Camera Info
    ##make ideal camera in rectified coordinate frame
    idealLeftCamera=CameraInfo()
    idealLeftCamera.height=stereoCalibration.inCalibrationData.meta.imgSize[0]
    idealLeftCamera.width=stereoCalibration.inCalibrationData.meta.imgSize[1]
    idealLeftCamera.distortion_model="plumb_bob" #TODO load based on distortion Model
    idealLeftCamera.D=[0,0,0,0,0]
    idealLeftCamera.K=intrin.Pl[0:3,0:3].flatten()
    idealLeftCamera.R=np.identity(3,np.float64).flatten()
    idealLeftCamera.P=intrin.Pl.flatten()
    idealLeftCamera.header.frame_id=transformNames[3]

    #im[y:y + h, x:x + w]
    overlapROI=getROIoverlap(intrin.lROI,intrin.rROI)
    ROIimage=copy.deepcopy(drawROI(intrin.lROI,intrin.rROI,overlapROI))


    pubROIimage=rospy.Publisher(rospy.get_name()+"/roiImage",Image,queue_size=3,latch=True)
    pubROIimage.publish(cvb.cv2_to_imgmsg(ROIimage))

    idealLeftCamera.roi.x_offset=overlapROI[1]
    idealLeftCamera.roi.y_offset=overlapROI[0]
    idealLeftCamera.roi.height=overlapROI[2]
    idealLeftCamera.roi.width=overlapROI[3]


    idealRightCamera=CameraInfo()
    idealRightCamera.height=stereoCalibration.inCalibrationData.meta.imgSize[0]
    idealRightCamera.width=stereoCalibration.inCalibrationData.meta.imgSize[1]
    idealRightCamera.distortion_model="plumb_bob"
    idealRightCamera.D=[0,0,0,0,0]

    idealRightCamera.K=intrin.Pr[0:3,0:3].flatten()
    idealRightCamera.R=np.identity(3,np.float64).flatten()
    idealRightCamera.P=intrin.Pr.flatten()
    idealRightCamera.header.frame_id=transformNames[4]

    idealRightCamera.roi.x_offset=overlapROI[1]
    idealRightCamera.roi.y_offset=overlapROI[0]
    idealRightCamera.roi.height=overlapROI[2]
    idealRightCamera.roi.width=overlapROI[3]


    cameraPublisherLeft=rospy.Publisher(rospy.get_name()+"/idealLeft/CameraInfo",CameraInfo,queue_size=10,latch=True)
    cameraPublisherRight = rospy.Publisher(rospy.get_name() + "/idealRight/CameraInfo", CameraInfo, queue_size=10,
                                      latch=True)
    cameraPublisherLeft.publish(idealLeftCamera)
    cameraPublisherRight.publish(idealRightCamera)
    print("published intrinsic Info")
    ########publish Rectification Maps

    lmapixPub=rospy.Publisher(rospy.get_name()+"/idealLeft/intX",Image,queue_size=3,latch=True)
    lmapiyPub = rospy.Publisher(rospy.get_name() + "/idealLeft/intY", Image, queue_size=3, latch=True)
    lmapfxPub = rospy.Publisher(rospy.get_name() + "/idealLeft/floatX", Image, queue_size=3, latch=True)
    lmapfyPub = rospy.Publisher(rospy.get_name() + "/idealLeft/floatY", Image, queue_size=3, latch=True)

    rmapixPub=rospy.Publisher(rospy.get_name()+"/idealRight/intX",Image,queue_size=3,latch=True)
    rmapiyPub = rospy.Publisher(rospy.get_name() + "/idealRight/intY", Image, queue_size=3, latch=True)
    rmapfxPub = rospy.Publisher(rospy.get_name() + "/idealRight/floatX", Image, queue_size=3, latch=True)
    rmapfyPub = rospy.Publisher(rospy.get_name() + "/idealRight/floatY", Image, queue_size=3, latch=True)


    lmapixPub.publish(cvb.cv2_to_imgmsg(lMap.intXMapping))
    lmapiyPub.publish(cvb.cv2_to_imgmsg(lMap.intYMapping))
    lmapfxPub.publish(cvb.cv2_to_imgmsg(lMap.floatXMapping))
    lmapfyPub.publish(cvb.cv2_to_imgmsg(lMap.floatYMapping))

    rmapixPub.publish(cvb.cv2_to_imgmsg(rMap.intXMapping))
    rmapiyPub.publish(cvb.cv2_to_imgmsg(rMap.intYMapping))
    rmapfxPub.publish(cvb.cv2_to_imgmsg(rMap.floatXMapping))
    rmapfyPub.publish(cvb.cv2_to_imgmsg(rMap.floatYMapping))
    print("published rectification Info")
    QPub=rospy.Publisher(rospy.get_name()+"/Q",Image,queue_size=3,latch=True)
    ###############
    ####in reference to issue bumblebee#3
    print(intrin.Q)
    modifiedQ=intrin.Q
    modifiedQ[3,2]=modifiedQ[3,2]*-1.0
    print("new",modifiedQ)
    QPub.publish(cvb.cv2_to_imgmsg(modifiedQ))
    print("bumblebee Configuration Server Running")
    if(publishData):
        rate=rospy.Rate(15)
        while(not rospy.is_shutdown()):
            time=rospy.Time.now()
            for i in chainTransforms:
                i.header.stamp=time
                pub.sendTransformMessage(i)
            rate.sleep()
    else:
        rospy.spin()
