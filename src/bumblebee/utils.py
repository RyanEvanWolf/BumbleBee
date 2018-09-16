import rospy
from math import pi,radians,degrees
import numpy as np
from tf.transformations import quaternion_from_matrix,quaternion_from_euler,quaternion_matrix,euler_from_matrix
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
import copy
import pickle

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

def composeR(roll,pitch,yaw,degrees=True,dict=True):
    if(degrees):
        q=quaternion_from_euler(radians(roll),
                                radians(pitch),
                                radians(yaw),'szxy')
    else:
        q=quaternion_from_euler(roll,
                                pitch,
                                yaw,'szxy')  
    return quaternion_matrix(q)[0:3,0:3] 

def createHomog(R=np.eye(3,dtype=np.float64),
                T=np.zeros((3,1),np.float64)):
    output=np.eye(4,dtype=np.float64)
    output[0:3,0:3]=R
    output[0:3,3]=T.reshape(3)
    return output

def composeTransform(R,T):
    ####H=[R -RT]
    ######[0   1]
    return createHomog(R,-R.dot(T))


def decomposeTransform(H):
    R=copy.deepcopy(H[0:3,0:3])
    T=-1*np.linalg.inv(R).dot(H[0:3,3])
    return createHomog(R,T)

def getMotion(H):
    Result={}
    angles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
    Result["Roll"]=57.2958*angles[0]
    Result["Pitch"]=57.2958*angles[1]
    Result["Yaw"]=57.2958*angles[2]

    Result["X"]=copy.deepcopy(H[0,3])
    Result["Y"]=copy.deepcopy(H[1,3])
    Result["Z"]=copy.deepcopy(H[2,3])
    return Result

def getPickledObject(fileName):
    f=open(fileName,"rb")
    a=copy.deepcopy(pickle.load(f))
    f.close()
    return a