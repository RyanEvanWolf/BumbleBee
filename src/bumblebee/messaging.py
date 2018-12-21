import rospy
from math import pi,radians,degrees
import numpy as np
from tf.transformations import quaternion_from_matrix,quaternion_from_euler,quaternion_matrix,euler_from_matrix
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
import copy
import pickle
import math
import os


def deserialHomography(arrayIn):
    outHomography=np.zeros((4,4),dtype=np.float64)
    row=0
    col=0
    for row in range(0,4):
        for col in range(0,4):
            outHomography[row,col]=arrayIn[row*4+col]
    return outHomography 


def msgFromTransform(inHomography):
    outTransform=Transform()
    T=inHomography[0:3,3]
    q=quaternion_from_matrix(inHomography)
    outTransform.translation.x=T[0]
    outTransform.translation.y=T[1]
    outTransform.translation.z=T[2]
    outTransform.rotation.x=q[0]
    outTransform.rotation.y=q[1]
    outTransform.rotation.z=q[2]
    outTransform.rotation.w=q[3]
    return outTransform 

