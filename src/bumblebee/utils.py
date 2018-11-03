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