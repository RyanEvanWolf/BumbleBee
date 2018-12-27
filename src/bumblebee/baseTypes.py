 
import cv2
import math
import time
import random
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image,PointCloud,ChannelFloat32

from geometry_msgs.msg import PoseStamped,TransformStamped,Point32

import copy
from bumblebee.stereo import getCameraSettingsFromServer,ROIfrmMsg
from bumblebee.utils import *
from bumblebee.motion import *
from bumblebee.camera import *

from visualization_msgs.msg import Marker,MarkerArray
import json
import msgpack
import msgpack_numpy as m

import networkx as nx

from tf import TransformListener,TransformBroadcaster

from tf.transformations import *

class stereoLandmarkEdge:
    def __init__(self,L,R,X=None,ID=-1):
        self.L=copy.deepcopy(L)
        self.R=copy.deepcopy(R)
        self.X=copy.deepcopy(X)
        self.ID=copy.deepcopy(ID)
    def getDisparity(self):
        return self.L[0,0]-self.R[0,0]
    def getEpiError(self):
        return self.L[1,0]-self.R[1,0]
    def getQtriangulate(self,Q):
        return reproject2(self.L,self.R,Q)
    def getReprojectedDifference(self,Pl,Pr):
        return getPredictedDiff(Pl,Pr,self.X,self.L,self.r)


class stereoFrame:
    def __init__(self):
        self.X=None
        self.R=None
        self.L=None

# class stereoFrame:
#     def __init__(self,lEdges=None,rEdges=None):
#         self.KPl=None
#         self.KPr=None
#         self.rawLdesc=None
#         self.rawRdesc=None
#         self.X=None
#         self.L=None
#         self.R=None

def randomPartition(ids,minParameters=7):
    indexes=range(len(ids))
    np.random.shuffle(indexes)
    parameterIndexes=sorted(indexes[:minParameters])
    testPointIdexes=sorted(indexes[minParameters:])
    return parameterIndexes,testPointIdexes
class slidingGraph(nx.DiGraph):
    def __init__(self,displayName="graph",frames=2,graph=None):
        if(graph is None):
            super(slidingGraph,self).__init__()
        else:
            super(slidingGraph,self).__init__(graph.copy())
        self.displayName=displayName
        self.nWindow=frames
        self.kSettings=getCameraSettingsFromServer(cameraType="subROI")
        roi=ROIfrmMsg(self.kSettings["lInfo"].roi)
        self.roiX,self.roiY,self.roiW,self.roiH=roi[0],roi[1],roi[2],roi[3]        
        self.nLandmarks=0
        self.nPoses=0

        
        self.debugTF =TransformBroadcaster()
        self.listener =TransformListener()

        self.debugPub=rospy.Publisher(displayName+"/debug/deltaPose",PoseStamped,queue_size=3)
        self.debugMap=rospy.Publisher(displayName+"/debug/Map",MarkerArray,queue_size=2)
        self.debugStereo=rospy.Publisher(displayName+"/debug/stereo",PointCloud,queue_size=2)
        self.debugCurrent=rospy.Publisher(displayName+"/debug/currentPose",PoseStamped,queue_size=2)
    def newPoseVertex(self,ID=None):
        if(ID is None):
            pId="p_"+str(self.nPoses).zfill(7)
        else:
            pId=ID
        msg=TransformStamped()
        msg.transform.rotation.w=1
        if(self.nPoses<1):
            msg.header.frame_id="world"
            msg.child_frame_id=self.displayName+"/p_"+str(self.nPoses).zfill(7)
        else:
            msg.header.frame_id=self.displayName+"/p_"+str(self.nPoses-1).zfill(7)
            msg.child_frame_id=self.displayName+"/p_"+str(self.nPoses).zfill(7)
        self.add_node(pId,t="Pose",c=self.nPoses,msg=msg)

        self.nPoses+=1
        return pId
    def newLandmarkVertex(self):
        
        pId="l_"+str(self.nLandmarks).zfill(7)
        self.add_node(pId,t="Landmark",c=self.nLandmarks)
        self.nLandmarks+=1
        return pId     
    def plotLandmark(self,outIMG,landmarkID,setPoses):
        '''
        setPoses must be in ascending order 000  00001 0002 
        '''

        poses=self.getLandmarkConnections(landmarkID)
        drawn=[]
        for p in poses:
            if(p in setPoses):
                data=self.edges[p,landmarkID]
                pt=(int(data["M"][0,0]),int(data["M"][1,0]))
                drawn.append(pt)
                if(p==setPoses[-1]):
                    cv2.circle(outIMG,pt,2,(0,255,0),1) 
                else:
                    cv2.circle(outIMG,pt,2,(130,255,0),1) 
        for d in range(len(drawn)-1):
            cv2.line(outIMG,drawn[d],drawn[d+1],(130,255,0))
   
        
    def getLandmarkConnections(self,landmarkID):
        e=self.in_edges(landmarkID)
        setTracks=[]
        for a in e:
            setTracks.append(a[0])
        return sorted(setTracks)
    #######################
    #######################
    ###calculate Stats
    ##################

    ########################################
    ####Motion Algorithms
    #########################################
    def svdRANSAC(self,sourceFrame,targetFrame):
        trackIDS=self.getLandmarkTracksAT(sourceFrame,targetFrame)


        maxIterations=1
        goodModel=0.8*self.nLandmarks
        bestFit=np.zeros((6,1))
        besterr=np.inf 
        bestInliers=[]
        minRMSerror=0.05
        
        previousX=np.zeros((4,len(trackIDS)))
        currentX=np.zeros((4,len(trackIDS)))
        
        count=0
        for k in trackIDS:
            previous=self.edges[sourceFrame,k]
            current=self.edges[targetFrame,k]

            previousX[:,count]=previous["X"].reshape(4)
            currentX[:,count]=current["X"].reshape(4)
            count+=1
        count=0
        msg=PoseStamped()
        msg.header.frame_id="world"
        while(count<maxIterations and besterr>minRMSerror):

            paramEstimateIndexes,testPointIndexes=randomPartition(trackIDS)
            
            motionHypothesis=rigid_transform_3D(currentX[0:3,paramEstimateIndexes],
                                                previousX[0:3,paramEstimateIndexes])

            
            msg.pose.orientation.x=motionHypothesis[0][0]
            msg.pose.orientation.y=motionHypothesis[0][1]
            msg.pose.orientation.z=motionHypothesis[0][2]
            msg.pose.orientation.w=motionHypothesis[0][3]

            msg.pose.position.x=motionHypothesis[1][0]
            msg.pose.position.y=motionHypothesis[1][1]
            msg.pose.position.z=motionHypothesis[1][2]




            self.nodes[targetFrame]["msg"].transform.rotation=msg.pose.orientation
            self.nodes[targetFrame]["msg"].transform.translation.x=motionHypothesis[1][0]
            self.nodes[targetFrame]["msg"].transform.translation.y=motionHypothesis[1][1]
            self.nodes[targetFrame]["msg"].transform.translation.z=motionHypothesis[1][2]
            count+=1
        self.debugPub.publish(msg)
        return msg
    #####################
    ###query algorithsm
    #####################
    def getPoseVertices(self): 
        s=sorted([x for x in self.nodes() if self.node[x]['t']=="Pose"])
        return s        
    def getLandmarkVertices(self):
        s=sorted([x for x in self.nodes() if self.node[x]['t']=="Landmark"])
        return s  
    def getLandmarksVisibleAT(self,poseID):
        return sorted([x[1] for x in self.edges() if x[0]==poseID])
    def getLandmarkTracksAT(self,sourcePose,targetPose):
        tracksSource=self.getLandmarksVisibleAT(sourcePose)
        tracksTarget=self.getLandmarksVisibleAT(targetPose)
        return sorted([x for x in tracksSource if x in tracksTarget])
    ######################
    ###display algorithms
    ######################
    def publishPoses(self):
        p=self.getPoseVertices()
        for pose in p:
            m=self.nodes[pose]["msg"]
            m.header.stamp=rospy.Time()
            self.debugTF.sendTransformMessage(m)
    def publishGlobalPoints(self):
        l=self.getLandmarkVertices()

        m=MarkerArray()
        for k in l:
            basePose=self.getLandmarkConnections(k)[0]
            newMarker=Marker()
            newMarker.header.frame_id=self.displayName+"/"+basePose
            newMarker.id=self.node[k]["c"]
            newMarker.type=2
            newMarker.action=0

            newMarker.pose.position.x=self.edges[basePose,k]["X"][0,0]
            newMarker.pose.position.y=self.edges[basePose,k]["X"][1,0]
            newMarker.pose.position.z=self.edges[basePose,k]["X"][2,0]


            newMarker.pose.orientation.w=1
            newMarker.scale.x=0.1
            newMarker.scale.y=0.1
            newMarker.scale.z=0.1
            newMarker.color.a=1
            newMarker.color.b=1
            m.markers.append(newMarker)

        self.debugMap.publish(m)      
    def publishLocalPoints(self,PoseID):
        activeLandmarks=self.getLandmarksVisibleAT(PoseID)
        msg=PointCloud()
        msg.header.frame_id=self.displayName+"/"+PoseID
        msg.header.stamp=rospy.Time()
        c=ChannelFloat32()
        c.name="rgb"
        c.values.append(255)
        c.values.append(0)
        c.values.append(0)

        for landmark in activeLandmarks:
            inPoint=Point32()
            inPoint.x=self.edges[PoseID,landmark]["X"][0,0]
            inPoint.y=self.edges[PoseID,landmark]["X"][1,0]
            inPoint.z=self.edges[PoseID,landmark]["X"][2,0]
            msg.points.append(inPoint)
            msg.channels.append(c)
        print("Points",len(msg.points))
        self.debugStereo.publish(msg)

    def publishCurrentPose(self):
        activePose=self.getPoseVertices()[-1]

        msg=PoseStamped()
        msg.header.frame_id="world"
        #msg.child_frame_id=self.displayName+"/"+activePose
        (t,r)=self.listener.lookupTransform('world',self.displayName+"/"+activePose, rospy.Time(0))
        
        msg.header.stamp=rospy.Time()

        msg.pose.position.x=t[0]
        msg.pose.position.y=t[1]
        msg.pose.position.z=t[2]

        msg.pose.orientation.x=r[0]
        msg.pose.orientation.y=r[1]
        msg.pose.orientation.z=r[2]
        msg.pose.orientation.w=r[3]

        self.debugCurrent.publish(msg)

        # self.listener

        # while(count<maxIterations and besterr>minRMSerror):
            

        #     paramEstimateIndexes,testPointIndexes=randomPartition(trackIDS)
            
        #     motionHypothesis=rigid_transform_3D(currentX[0:3,paramEstimateIndexes],
        #                                         previousX[0:3,paramEstimateIndexes])

            
        #     msg.pose.orientation.x=motionHypothesis[0][0]
        #     msg.pose.orientation.y=motionHypothesis[0][1]
        #     msg.pose.orientation.z=motionHypothesis[0][2]
        #     msg.pose.orientation.w=motionHypothesis[0][3]

        #     msg.pose.position.x=motionHypothesis[1][0]
        #     msg.pose.position.y=motionHypothesis[1][1]
        #     msg.pose.position.z=motionHypothesis[1][2]




        #     self.nodes[targetFrame]["msg"].transform.rotation=msg.pose.orientation
        #     self.nodes[targetFrame]["msg"].transform.translation.x=motionHypothesis[1][0]
        #     self.nodes[targetFrame]["msg"].transform.translation.y=motionHypothesis[1][1]
        #     self.nodes[targetFrame]["msg"].transform.translation.z=motionHypothesis[1][2]
        #     count+=1
        # self.debugPub.publish(msg)

class slidingWindow(object):
    def __init__(self,cameraSettings=None,frames=2):
        if(cameraSettings is None):
            self.kSettings=None
        else:
            self.kSettings=copy.deepcopy(cameraSettings)
        self.X=np.zeros((0,0),dtype=np.float64) ###[Pose0 Pose1 Pose2|landmarkA landmarkB landmarkC ...]
        self.M=[]
        self.tracks=[]
        self.inliers=[]
        self.nLandmarks=0
        self.nPoses=frames
    ###########
    ##admin functions
    ###############
    def serializeWindow(self):
        binDiction={}
        binDiction["kSettings"]=pickle.dumps(self.kSettings)
        binDiction["M"]=[]
        for i in self.M:
            binDiction["M"].append(msgpack.packb(i,default=m.encode))
        binDiction["X"]=msgpack.packb(self.X,default=m.encode)
        binDiction["inliers"]=msgpack.dumps(self.inliers)
        binDiction["tracks"]=msgpack.dumps(self.tracks)
        binDiction["nLandmarks"]=self.nLandmarks
        binDiction["nPoses"]=self.nPoses
        return msgpack.dumps(binDiction)
    def deserializeWindow(self,data):
        intern=msgpack.loads(data)
        self.kSettings=pickle.loads(intern["kSettings"])
        self.X=msgpack.unpackb(intern["X"],object_hook=m.decode)
        self.M=[]
        for i in intern["M"]:
            self.M.append(msgpack.unpackb(i,object_hook=m.decode))
        self.inliers=msgpack.loads(intern["inliers"])
        self.tracks=msgpack.loads(intern["tracks"])
        self.nLandmarks=intern["nLandmarks"]
        self.nPoses=intern["nPoses"]
    ################
    ##future functions
    def addLandmark(X,measurements,trackIndexes=None):
        self.X=np.vstack(X)
    def addPose(Xpose):
        pass
    ##################
    def getPoseX(self,poseIndex):
            if(poseIndex==0):
                return np.zeros((6,1))
            else:
                realIndex=poseIndex-1
                return self.X[6*realIndex:6*realIndex +6,0].reshape(6,1)
    def getLandmarkX(self,landmarkIndex):
        return self.X[6 +landmarkIndex*4:6+landmarkIndex*4 +4,0].reshape(4,1)
    def reprojectLandmark(self,landmarkIndex):
        '''
        estimate reprojections from measurement matrix.
        Triangulations are relative to their respective coordinate frames
        '''
        measVect=np.zeros((4,len(self.tracks[landmarkIndex])))
        measVect[3,:]=np.ones((1,len(self.tracks[landmarkIndex])))
        measVect[0:2,:]=self.M[landmarkIndex][0:2,:]
        measVect[2,:]=self.M[landmarkIndex][0,:]-self.M[landmarkIndex][2,:]
        xreproject=self.kSettings["Q"].dot(measVect)
        xreproject/=xreproject[3,:]
        return xreproject
    def getLandmarkRMS(self,landmarkIndex):

        #print(np.sum(self.getPixelError(landmarkIndex)**2)))
        return np.sqrt((self.getPixelError(landmarkIndex)**2).mean())
    def getAllLandmarkRMS(self):
        rmsError=[]
        for i in range(self.nLandmarks):
            rmsError.append(self.getLandmarkRMS(i))
        return rmsError
    def getPixelError(self,landmarkIndex):
        Mest=np.zeros((4,len(self.tracks[landmarkIndex])))
        for i in range(0,len(self.tracks[landmarkIndex])):
            lCam,rCam=self.composeCamera(self.tracks[landmarkIndex][i])
            lpred=lCam.dot(self.getLandmarkX(landmarkIndex))
            lpred/=lpred[2,0]
            Mest[0:2,i]=lpred[0:2,0]
            rpred=rCam.dot(self.getLandmarkX(landmarkIndex))
            rpred/=rpred[2,0]
            Mest[2:4,i]=rpred[0:2,0]
        # print(Mest)
        return Mest-self.M[landmarkIndex]
    def getWindowRMS(self):
        errorMatrix=[]
        for landmarkIndex in range(self.nLandmarks):
            errorMatrix.append(self.getPixelError(landmarkIndex).flatten())
        eVect=np.concatenate([np.array(i) for i in errorMatrix])
        return np.sqrt((eVect**2).mean())
    def composeCamera(self,poseIndex):
        homog=getH(self.getPoseX(poseIndex))
        return self.kSettings["Pl"].dot(homog),self.kSettings["Pr"].dot(homog)
    def getNinliers(self):
        return np.count(self.inliers)
    def getSubset(self,landmarkIndexes):

        subset=slidingWindow(self.kSettings)
        subset.nLandmarks=len(landmarkIndexes)
        subset.X=np.zeros((6*(self.nPoses-1)+4*len(landmarkIndexes),1),dtype=np.float64) ###[Pose0 Pose1 Pose2|landmarkA landmarkB landmarkC ...]
        for i in range(0,self.nPoses-1):
            subset.X[6*i:6*i+6,0]=self.getPoseX(i+1).reshape(6)
        for i in range(0,len(landmarkIndexes)):
            subset.tracks.append(self.tracks[i])
            subset.X[6 +4*i:6+4*i+4,0]=self.getLandmarkX(landmarkIndexes[i]).reshape(4)
            subset.M.append(self.M[landmarkIndexes[i]])

        return subset


class stereoEdge:
    def __init__(self,X,L,R,frameNumber=-1):
        self.X=copy.deepcopy(X)
        self.L=copy.deepcopy(L)
        self.R=copy.deepcopy(R)
        self.frameID=frameNumber
    def __str__(self):
        ans=""
        ans+="ID:"+str(self.frameID)+"\n"
        ans+="X:"+str(np.transpose(self.X))+"\n"
        ans+="L:"+str(np.transpose(self.L))+"\n"
        ans+="R:"+str(np.transpose(self.R))+"\n"
        return ans
class trackEdge:
    def __init__(self):
        self.tracks=[]
        self.motion=[]
    
class interFrameEdge:
    def __init__(self):
        self.currentEdges=[]
        self.Tracks=[]
        self.previousEdges=[]
        self.motion=None
    def getCurrentX(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.X)
        return result
    def getPreviousX(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.X)
        return result   
    def getPreviousL(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.L)
        return result      
    def getPreviousR(self):
        result=[]
        for a in self.previousEdges:
            result.append(a.R)
        return result        
    def getCurrentL(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.L)
        return result      
    def getCurrentR(self):
        result=[]
        for a in self.currentEdges:
            result.append(a.R)
        return result     


# class motionEdge:
#     def __init__(self,H):
#         self.H=copy.deepcopy(H)
#         self.angles=None
#         self.Tc=None
#         self.R=None
#         self.H=None
#         self.Htransform=None
#     def getR(self):
#         return copy.deepcopy(self.H[0:3,0:3])
#     def getT(self):
#         return copy.deepcopy(self)
#     def getCoordinateTransform(self):
#         return composeTransform()
#     def invert(self,H=None):
#         if(H==None):
#             self.H=np.linalg.inv(self.H.dot(H)

#     def initH(self,H):
#         self.H=copy.deepcopy(H)
#         self.R=copy.deepcopy(H[0:3,0:3])
#         tmpAngles=copy.deepcopy(euler_from_matrix(H[0:3,0:3],'szxy'))
#         self.angles=np.zeros((1,3))
#         self.angles[0,0]=degrees(tmpAngles[0])
#         self.angles[0,1]=degrees(tmpAngles[1])
#         self.angles[0,2]=degrees(tmpAngles[2])

#         self.Tc=np.zeros((3,1))
#         self.Tc[0,0]=self.H[0,3]
#         self.Tc[1,0]=self.H[1,3]
#         self.Tc[2,0]=self.H[2,3]
#         self.Htransform=composeTransform(self.R,self.Tc)
#     def initData(self,angles=np.zeros((1,3)),coords=np.zeros((3,1))):
#         self.angles=copy.deepcopy(angles) ###in degrees
#         self.Tc=copy.deepcopy(coords)  ###in meters
#         self.R=composeR(self.angles[0,0],self.angles[0,1],self.angles[0,2])##assume degrees
#         self.H=createHomog(self.R,self.Tc)
#         self.Htransform=composeTransform(self.R,self.Tc)
#     def getMotion(self):
#         dictionary={}
#         return dictionary
