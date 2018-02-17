#!/usr/bin/env python
import rospy
import sys
import os
import numpy as np
import cv2

import scipy.stats.mstats as stat

import matplotlib.pyplot as plt
from bumblebee_calibration import *

import pickle
import copy
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
import tf.transformations

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
    if(len(sys.argv)>2):
        nodeName=sys.argv[2]
    rospy.init_node(nodeName)
    print("Load from "+calibrationDirectory)
    lMap = pickle.load(open(calibrationDirectory + "/left_Map.p", "rb"))
    print("Left Mapping Loaded")
    rMap = pickle.load(open(calibrationDirectory + "/right_Map.p", "rb"))
    print("Right Mapping Loaded")
    extrin = pickle.load(open(calibrationDirectory + "/extrinsicCalibration.p", "rb"))
    print("Camera Extrinsics Loaded")
    intrin = pickle.load(open(calibrationDirectory + "/intrinsicCalibration.p", "rb"))
    print("Camera Intrinsics Loaded")
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



    print("bumblebee Configuration Server Running")
    rate=rospy.Rate(15)
    while(not rospy.is_shutdown()):
        time=rospy.Time.now()
        for i in chainTransforms:
            i.header.stamp=time
            pub.sendTransformMessage(i)
        rate.sleep()

