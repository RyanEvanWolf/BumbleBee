import rospy
from math import pi,radians,degrees
import numpy as np
from tf.transformations import quaternion_from_matrix,quaternion_from_euler,quaternion_matrix,euler_from_matrix
from geometry_msgs.msg import TransformStamped,Transform,Quaternion,Vector3
import copy
import pickle
import math
import os

def createDir(newDir):
    if(not os.path.exists(newDir)):
        os.makedirs(newDir)


def np2cvDisplay(npIn):
    return (int(npIn[0,0]),int(npIn[1,0]))



def rmsError(xPred,X):
    return np.sqrt(((xPred-X)**2).mean())

def sumSquareError(xPred,X):
    return np.sqrt(((xPred-X)**2).sum())

# def composeR(roll,pitch,yaw,degrees=True,dict=True):
#     if(degrees):
#         q=quaternion_from_euler(radians(roll),
#                                 radians(pitch),
#                                 radians(yaw),'szxy')
#     else:
#         q=quaternion_from_euler(roll,
#                                 pitch,
#                                 yaw,'szxy')  
#     return quaternion_matrix(q)[0:3,0:3] 

# def createHomog(R=np.eye(3,dtype=np.float64),
#                 T=np.zeros((3,1),np.float64)):
#     output=np.eye(4,dtype=np.float64)
#     output[0:3,0:3]=R
#     output[0:3,3]=T.reshape(3)
#     return output

# def composeTransform(R,T):
#     ####H=[R -RT]
#     ######[0   1]
#     return createHomog(R,-R.dot(T))


# def decomposeTransform(H):
#     R=copy.deepcopy(H[0:3,0:3])
#     T=-1*np.linalg.inv(R).dot(H[0:3,3])
#     return createHomog(R,T)

# def getMotion(H):
#     Result={}
#     angles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
#     Result["Roll"]=57.2958*angles[0]
#     Result["Pitch"]=57.2958*angles[1]
#     Result["Yaw"]=57.2958*angles[2]

#     Result["X"]=copy.deepcopy(H[0,3])
#     Result["Y"]=copy.deepcopy(H[1,3])
#     Result["Z"]=copy.deepcopy(H[2,3])u
#     return Result

def getPickledObject(fileName):
    f=open(fileName,"rb")
    a=copy.deepcopy(pickle.load(f))
    f.close()
    return a

def to_precision(x,p):
    """
    returns a string representation of x formatted with a precision of p

    Based on the webkit javascript implementation taken from here:
    https://code.google.com/p/webkit-mirror/source/browse/JavaScriptCore/kjs/number_object.cpp
    """

    x = float(x)

    if x == 0.:
        return "0." + "0"*(p-1)

    out = []

    if x < 0:
        out.append("-")
        x = -x

    e = int(math.log10(x))
    tens = math.pow(10, e - p + 1)
    n = math.floor(x/tens)

    if n < math.pow(10, p - 1):
        e = e -1
        tens = math.pow(10, e - p+1)
        n = math.floor(x / tens)

    if abs((n + 1.) * tens - x) <= abs(n * tens -x):
        n = n + 1

    if n >= math.pow(10,p):
        n = n / 10.
        e = e + 1

    m = "%.*g" % (p, n)

    if e < -2 or e >= p:
        out.append(m[0])
        if p > 1:
            out.append(".")
            out.extend(m[1:p])
        out.append('e')
        if e > 0:
            out.append("+")
        out.append(str(e))
    elif e == (p -1):
        out.append(m)
    elif e >= 0:
        out.append(m[:e+1])
        if e+1 < len(m):
            out.append(".")
            out.extend(m[e+1:])
    else:
        out.append("0.")
        out.extend(["0"]*-(e+1))
        out.append(m)

    return "".join(out)