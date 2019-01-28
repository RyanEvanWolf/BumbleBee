 
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
from bumblebee.stereo import getCameraSettingsFromServer,ROIfrmMsg,rmsReprojectionError,getProjections
from bumblebee.utils import *
from bumblebee.motion import *
from bumblebee.camera import *

from bumblebee.srv import saveGraph,saveGraphRequest,saveGraphResponse

from visualization_msgs.msg import Marker,MarkerArray
import json
import msgpack
import msgpack_numpy as m

import networkx as nx

from tf import TransformListener,TransformBroadcaster

from tf.transformations import *


from std_msgs.msg import ColorRGBA,Float32



import matplotlib.pyplot as plt

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





class basicGraph(nx.DiGraph):
    def __init__(self,newCamera=True):
        super(basicGraph,self).__init__()
        if(newCamera):
            self.kSettings=getCameraSettingsFromServer(cameraType="subROI")
            roi=ROIfrmMsg(self.kSettings["lInfo"].roi)
            self.roiX,self.roiY,self.roiW,self.roiH=roi[0],roi[1],roi[2],roi[3]        
        self.nLandmarks=0
        self.nPoses=0
        self.descriptorLength=16

    def newPoseVertex(self,ID=None):
        if(ID is None):
            pId="p_"+str(self.nPoses).zfill(7)
        else:
            pId=ID
        msg=TransformStamped()
        msg.transform.rotation.w=1
        if(self.nPoses<1):
            msg.header.frame_id="world"
            msg.child_frame_id="p_"+str(self.nPoses).zfill(7)
        else:
            msg.header.frame_id="p_"+str(self.nPoses-1).zfill(7)
            msg.child_frame_id="p_"+str(self.nPoses).zfill(7)
        self.add_node(pId,t="Pose",c=self.nPoses,msg=msg)

        self.nPoses+=1
        return pId
    def newLandmarkVertex(self):
        
        pId="l_"+str(self.nLandmarks).zfill(7)
        self.add_node(pId,t="Landmark",c=self.nLandmarks)
        self.nLandmarks+=1
        return pId 
    def plotLandmark(self,outIMG,landmarkID,setPoses,colour=(0,255,255)):
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
                    cv2.circle(outIMG,pt,2,colour,1) 
                # else:
                #     cv2.circle(outIMG,pt,2,(0,0,255),1) 
        for d in range(len(drawn)-1):
            cv2.line(outIMG,drawn[d],drawn[d+1],colour)

        
    def getLandmarkConnections(self,landmarkID):
        e=self.in_edges(landmarkID)
        setTracks=[]
        for a in e:
            setTracks.append(a[0])

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
        print("RANSAC @",targetFrame)
        trackIDS=self.getLandmarkTracksAT(sourceFrame,targetFrame)

        startTime=time.time()
        maxIterations=400
        goodModel=0.8*self.nLandmarks
        bestFit=[[0,0,0,1],[0,0,0]]
        besterr=np.inf 
        bestInliers=[]
        motionRMS=[]
        minRMSerror=0.3

        inlierThreshold=1

        count=0
        minimumInliers=int(0.2*len(trackIDS))

        Xmatrix,Mmatrix=self.getLandmarkTrackDataAT(sourceFrame,targetFrame)
        count=0
        msg=PoseStamped()
        msg.header.frame_id="world"
        msg.pose.orientation.w=1

        while(count<maxIterations and besterr>minRMSerror):
            paramEstimateIndexes,testPointIndexes=randomPartition(trackIDS,3)
            try:
                motionHypothesis=rigid_transform_3D(Xmatrix[0:3,paramEstimateIndexes],
                                                    Xmatrix[4:7,paramEstimateIndexes])



                R=quaternion_matrix(motionHypothesis[0])
                T=np.zeros((3,1))
                
                T[0,0]=motionHypothesis[1][0]
                T[1,0]=motionHypothesis[1][1]
                T[2,0]=motionHypothesis[1][2]

                H=createHomog(R[0:3,0:3],T)
                predictions=getProjections([self.kSettings["Pl"],self.kSettings["Pr"],
                                            self.kSettings["Pl"].dot(H),self.kSettings["Pr"].dot(H)],
                            Xmatrix[0:4,testPointIndexes])
                e=predictions-Mmatrix[:,testPointIndexes]
                inlierIndexes=[]
                inlierRMS=[]
                for i in range(len(testPointIndexes)):
                    rms=np.sqrt((e[:,i].flatten()**2).mean())
                    if(rms<inlierThreshold):
                        inlierIndexes.append(testPointIndexes[i])
                        inlierRMS.append(rms)



                if(len(inlierIndexes)>len(bestInliers)and(len(inlierIndexes)>minimumInliers)):
                    
                    ############
                    ###new model
                    #plt.plot(inlierRMS)

                    # betterHypothesis=rigid_transform_3D(Xmatrix[0:3,inlierIndexes],
                    #                                  Xmatrix[4:7,inlierIndexes])
                    # R=quaternion_matrix( betterHypothesis[0])
                    # T=np.zeros((3,1))
                    
                    # T[0,0]= betterHypothesis[1][0]
                    # T[1,0]= betterHypothesis[1][1]
                    # T[2,0]= betterHypothesis[1][2]

                    H=createHomog(R[0:3,0:3],T)
                    predictions=getProjections([self.kSettings["Pl"],self.kSettings["Pr"],
                                                self.kSettings["Pl"].dot(H),self.kSettings["Pr"].dot(H)],
                                Xmatrix[0:4,inlierIndexes])
                    e=predictions-Mmatrix[:,inlierIndexes]
                    RMS=np.sqrt((e.flatten()**2).mean())
                    secondRMS=[]
                    for i in range(len(inlierIndexes)):
                        rms=np.sqrt((e[:,i].flatten()**2).mean())
                        secondRMS.append(rms)
                    motionRMS=secondRMS
                    #########
                    ##convert into transform format
                    #######
                    H=np.linalg.inv(H)
                    q=quaternion_from_matrix(H)
                    C=H[0:3,0:3].dot(H[0:3,3].reshape(3,1))
                    cx=C[0,0]
                    cy=C[1,0]
                    cz=C[2,0]
                    betterHypothesis=[q,[cx,cy,cz]]

                    bestFit=betterHypothesis
                    besterr=RMS
                    bestInliers=inlierIndexes 
                    #print("Better Model",besterr,len(bestInliers)) 

                    #plt.plot(secondRMS,'r')
                    #plt.show()
                
            except Exception as e:
                print("motion Fail",e)

            count+=1
        startTime=time.time()-startTime
        print("----RESULT",besterr,len(bestInliers))
        msg.pose.orientation.x=bestFit[0][0]
        msg.pose.orientation.y=bestFit[0][1]
        msg.pose.orientation.z=bestFit[0][2]
        msg.pose.orientation.w=bestFit[0][3]###display is inverse of calculation

        msg.pose.position.x=bestFit[1][0]
        msg.pose.position.y=bestFit[1][1]
        msg.pose.position.z=bestFit[1][2]

        self.nodes[targetFrame]["msg"].transform.rotation=msg.pose.orientation
        self.nodes[targetFrame]["msg"].transform.translation.x=msg.pose.position.x
        self.nodes[targetFrame]["msg"].transform.translation.y=msg.pose.position.y
        self.nodes[targetFrame]["msg"].transform.translation.z=msg.pose.position.z
        self.nodes[targetFrame]["inlierRMS"]=motionRMS
        self.nodes[targetFrame]["inliers"]=[trackIDS[x] for x in bestInliers]#trackIDS[bestInliers]
        self.nodes[targetFrame]["time"]=startTime
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
    def getLandmarkTrackDataAT(self,sourcePose,targetPose):
        TrackSet=self.getLandmarkTracksAT(sourcePose,targetPose)

        X=np.zeros((4*2,len(TrackSet)))
        M=np.zeros((3*2,len(TrackSet)))

        for k in range(len(TrackSet)):
            X[0:4,k]=self.edges[(sourcePose,TrackSet[k])]["X"].reshape(4)
            X[4:8,k]=self.edges[(targetPose,TrackSet[k])]["X"].reshape(4)
            M[0:3,k]=self.edges[(sourcePose,TrackSet[k])]["M"][0:3,0].reshape(3)
            M[3:6,k]=self.edges[(targetPose,TrackSet[k])]["M"][0:3,0].reshape(3)
        return X,M
    def getLandmarkDataAT(self,setPoses,landmarkID):
        X =np.zeros((4,len(setPoses)))
        M=np.zeros((3,len(setPoses)))
        
        for p in range(len(setPoses)):
            X[:,p]=self.edges[(setPoses[p],landmarkID)]["X"].reshape(4)
            M[:,p]=self.edges[(setPoses[p],landmarkID)]["M"][0:3,0].reshape(3)
        return X,M
    def getDescriptors(self,poseID):
        activeLandmarks=self.getLandmarksVisibleAT(poseID)
        lDesc=np.zeros((len(activeLandmarks),self.descriptorLength),np.uint8)
        rDesc=np.zeros((len(activeLandmarks),self.descriptorLength),np.uint8)
        for i in range(0,len(activeLandmarks)):
            lDesc[i,:]=copy.deepcopy(self.edges[poseID,activeLandmarks[i]]["Dl"])#.reshape(self.descriptorLength))
            rDesc[i,:]=copy.deepcopy(self.edges[poseID,activeLandmarks[i]]["Dr"])#.reshape(self.descriptorLength))
        return lDesc,rDesc

    #############################
    ###internal graphing algorithms
    ########################
    ###graphing Options
    def getDeltaMotion(self):
        Cz=[]
        Cy=[]
        Cx=[]

        roll=[]
        pitch=[]
        yaw=[]  
   
        for p in self.getPoseVertices()[1:]:
            msg=self.node[p]["msg"]
            Cx.append(msg.transform.translation.x)
            Cy.append(msg.transform.translation.y)
            Cz.append(msg.transform.translation.z)
            q=[msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w]
            R=euler_from_quaternion(q,'szxy')
            roll.append(degrees(R[0]))
            pitch.append(degrees(R[1]))
            yaw.append(degrees(R[2]))  
        return roll,pitch,yaw,Cx,Cy,Cz
    def getTotalTracks(self):
        inliers=[]
        poseNames=self.getPoseVertices()
        for p in range(1,len(poseNames)):
            inliers.append(len(self.getLandmarkTracksAT(poseNames[p],
                                                    poseNames[p-1])))
        return inliers
    def getInlierMotion(self):
        inliers=[]
        for p in self.getPoseVertices()[1:]:
            msg=self.node[p]
            inliers.append(len(msg["inliers"]))
        return inliers       
    def getInlierRatio(self):
        inliers=[]
        poseNames=self.getPoseVertices()

        for p in range(1,len(poseNames)):
            print(poseNames[p],poseNames[p-1],p/len(poseNames))
            msg=self.node[poseNames[p]]
            trackIds=self.getLandmarkTracksAT(poseNames[p],
                                                    poseNames[p-1])                                
            try:
                inliers.append(len(msg["inliers"])/float(len(trackIds)))
            except:
                inliers.append(len(msg["inliers"]))
        return inliers        
    def getInlierRMS(self):
        RMS=[]
        for p in self.getPoseVertices()[1:]:
            msg=self.node[p]
            RMS.append(np.mean(msg["inlierRMS"])   )     
        return RMS
    def getMotionTime(self):
        mTime=[]
        for p in self.getPoseVertices()[1:]:
            msg=self.node[p]
            mTime.append(np.mean(msg["time"])   )     
        return mTime       
class slidingGraph(object):
    def __init__(self,displayName="graph",newCam=True):
        self.displayName=displayName        
        self.debugTF =TransformBroadcaster()
        self.listener =TransformListener()         
        self.cvb=CvBridge()
        self.G=basicGraph(newCam)
        self.nWindow=2



        self.debugDeltaPose=rospy.Publisher(displayName+"/debug/deltaPose",PoseStamped,queue_size=1)
        self.debugDeltaStereo=rospy.Publisher(displayName+"/debug/deltaStereo",PointCloud,queue_size=1)
        self.debugMap=rospy.Publisher(displayName+"/debug/Map",MarkerArray,queue_size=2)
        self.debugCurrent=rospy.Publisher(displayName+"/debug/currentPose",PoseStamped,queue_size=2)
        self.debugMatches=rospy.Publisher(displayName+"/debug/Tracks",Image,queue_size=4)

        self.debugInliers=rospy.Publisher(displayName+"/debug/nInliers",Float32,queue_size=4)
        self.debugRatio=rospy.Publisher(displayName+"/debug/ratio",Float32,queue_size=4)

        self.outSrv=rospy.Service(displayName+"/control/save",saveGraph,self.saveGraph)
    ###################
    ###MatPlotLib info
    def graphDeltaMotion(self):
        Rtheta=[]
        Cz=[]
        Cy=[]
        Cx=[]

        roll=[]
        pitch=[]
        yaw=[]
        for p in self.G.getPoseVertices()[1:]:
            msg=self.G.node[p]["msg"]
            Cx.append(msg.transform.translation.x)
            Cy.append(msg.transform.translation.y)
            Cz.append(msg.transform.translation.z)
            q=[msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w]
            R=euler_from_quaternion(q,'szxy')
            roll.append(degrees(R[0]))
            pitch.append(degrees(R[1]))
            yaw.append(degrees(R[2]))

        fig,(ax1,ax2)=plt.subplots(nrows=2, ncols=1,sharex=True,squeeze=True)
        fig.suptitle(self.displayName)
        ax1.plot(Cx,label="X")
        ax1.plot(Cy,label="Y")
        ax1.plot(Cz,label="Z")
        ax1.legend()
        ax2.plot(roll,label="roll")
        ax2.plot(pitch,label="pitch")
        ax2.plot(yaw,label="yaw")
        ax2.legend()
    def getCumulativeError(self,compareName):

        roll=[]
        pitch=[]
        yaw=[]

        Tx=[]
        Ty=[]
        Tz=[]

        for i in self.G.getPoseVertices()[1:]:
            (trans,rot)=self.listener.lookupTransform("world",self.displayName+"/"+i,rospy.Time(0))
            (transIdeal,rotIdeal)=self.listener.lookupTransform("world",compareName+"/"+i,rospy.Time(0))

            R=euler_from_quaternion(rot,'szxy')
            Rideal=euler_from_quaternion(rotIdeal,'szxy')
            roll.append(abs(degrees(R[0]-Rideal[0])))
            pitch.append(abs(degrees(R[1]-Rideal[1])))
            yaw.append(abs(degrees(R[2]-Rideal[2])))

            Tx.append(abs(trans[0]-transIdeal[0]))
            Ty.append(abs(trans[1]-transIdeal[1]))
            Tz.append(abs(trans[2]-transIdeal[2]))
        return roll,pitch,yaw,Tx,Ty,Tz
    ###################3
    ##ROS publishing functionsnlier[j]/float(tracks[j]))
    def publishPoses(self):
        p=self.G.getPoseVertices()
        for pose in p:
            m=copy.deepcopy(self.G.nodes[pose]["msg"])
            m.header.stamp=rospy.Time()
            if(m.header.frame_id != "world"):
                m.header.frame_id=self.displayName+'/'+m.header.frame_id
            m.child_frame_id=self.displayName+'/'+ m.child_frame_id
            m.header.stamp=rospy.Time()
            self.debugTF.sendTransformMessage(m)
    def publishGlobalPoints(self,colour=ColorRGBA(0,0,1,0.5)):
        l=self.G.getLandmarkVertices()
        totalDisp=int(0.8*len(l))   
        displayIndexes=range(len(l))
        np.random.shuffle(displayIndexes)
        displayIndexes=displayIndexes[:totalDisp]
        
       # len(l)))#[:totalDisp]
        #print(len(inLand))
        m=MarkerArray()
        errors=0
        for k in displayIndexes:
            try:
                basePose=self.G.getLandmarkConnections(l[k])[0]
                newMarker=Marker()
                newMarker.header.frame_id=self.displayName+"/"+basePose
                newMarker.id=self.G.node[l[k]]["c"]
                newMarker.type=2
                newMarker.action=0

                newMarker.pose.position.x=self.G.edges[basePose,l[k]]["X"][0,0]
                newMarker.pose.position.y=self.G.edges[basePose,l[k]]["X"][1,0]
                newMarker.pose.position.z=self.G.edges[basePose,l[k]]["X"][2,0]


                newMarker.pose.orientation.w=1
                newMarker.scale.x=0.025
                newMarker.scale.y=0.025
                newMarker.scale.z=0.025
                newMarker.color=colour
                m.markers.append(newMarker)
            except Exception as e:
                errors+=1
        print("errors",errors)

        self.debugMap.publish(m)   
    def publishLocalPoints(self,PoseID,delta=True):
        activeLandmarks=self.G.getLandmarksVisibleAT(PoseID)
        msg=PointCloud()
        if(delta):
            msg.header.frame_id="world"
        else:
            msg.header.frame_id=self.displayName+"/"+PoseID
        msg.header.stamp=rospy.Time()
        c=ChannelFloat32()
        c.name="rgb"
        c.values.append(255)
        c.values.append(0)
        c.values.append(0)
        for landmark in activeLandmarks:
            inPoint=Point32()
            inPoint.x=self.G.edges[PoseID,landmark]["X"][0,0]
            inPoint.y=self.G.edges[PoseID,landmark]["X"][1,0]
            inPoint.z=self.G.edges[PoseID,landmark]["X"][2,0]
            msg.points.append(inPoint)
            msg.channels.append(c)
        self.debugDeltaStereo.publish(msg)
    def publishInlierOutlierTracks(self,sourceFrame,targetFrame,img=None):
            if(img is None):
                img=np.zeros((768,1024,3))
            trackids=self.G.getLandmarkTracksAT(sourceFrame,targetFrame)
            for a in trackids:
                self.G.plotLandmark(img,a,[sourceFrame,targetFrame],(0,0,255))
            inlierIds=self.G.nodes[targetFrame]["inliers"]
            print(len(inlierIds),len(trackids))
            for a in inlierIds:
                self.G.plotLandmark(img,a,[sourceFrame,targetFrame],(255,255,0))
            self.debugMatches.publish(self.cvb.cv2_to_imgmsg(img))

            nInlier=Float32()
            nInlier.data=len(inlierIds)
            ratio=Float32()

            ratio.data=len(inlierIds)/float(len(trackids))
            self.debugInliers.publish(nInlier)
            self.debugRatio.publish(ratio)#=rospy.Publisher(displayName+"/debug/ratio",Float32,queue_size=4)
    def saveGraph(self,req):
        outFile=req.outputFileName
        with open(outFile,'wb') as f:
            pickle.dump(self.G,f)
        print("Graph save To " +outFile)
        return saveGraphResponse()
# class slidingGraph(nx.DiGraph):
#     def __init__(self,displayName="graph",frames=2,graph=None):
        
#         if(graph is None):
#             super(slidingGraph,self).__init__()
#         else:
#             super(slidingGraph,self).__init__(graph.copy())
#         self.displayName=displayName
#         
#         self.kSettings=getCameraSettingsFromServer(cameraType="subROI")
#         roi=ROIfrmMsg(self.kSettings["lInfo"].roi)
#         self.roiX,self.roiY,self.roiW,self.roiH=roi[0],roi[1],roi[2],roi[3]        
#         self.nLandmarks=0
#         self.nPoses=0

#         self.cvb=CvBridge()
#         self.debugTF =TransformBroadcaster()
#         self.listener =TransformListener()

#         self.debugDeltaPose=rospy.Publisher(displayName+"/debug/deltaPose",PoseStamped,queue_size=1)
#         self.debugDeltaStereo=rospy.Publisher(displayName+"/debug/deltaStereo",PointCloud,queue_size=1)
#         self.debugMap=rospy.Publisher(displayName+"/debug/Map",MarkerArray,queue_size=2,latch=True)
#         self.debugStereo=rospy.Publisher(displayName+"/debug/stereo",PointCloud,queue_size=2)
#         self.debugCurrent=rospy.Publisher(displayName+"/debug/currentPose",PoseStamped,queue_size=2)
#         self.debugMatches=rospy.Publisher(displayName+"/debug/Tracks",Image,queue_size=4)

#         self.debugInliers=rospy.Publisher(displayName+"/debug/nInliers",Float32,queue_size=4)
#         self.debugRatio=rospy.Publisher(displayName+"/debug/ratio",Float32,queue_size=4)
#     def newPoseVertex(self,ID=None):
#         if(ID is None):
#             pId="p_"+str(self.nPoses).zfill(7)
#         else:
#             pId=ID
#         msg=TransformStamped()
#         msg.transform.rotation.w=1
#         if(self.nPoses<1):
#             msg.header.frame_id="world"
#             msg.child_frame_id=self.displayName+"/p_"+str(self.nPoses).zfill(7)
#         else:
#             msg.header.frame_id=self.displayName+"/p_"+str(self.nPoses-1).zfill(7)
#             msg.child_frame_id=self.displayName+"/p_"+str(self.nPoses).zfill(7)
#         self.add_node(pId,t="Pose",c=self.nPoses,msg=msg)

#         self.nPoses+=1
#         return pId
#     def newLandmarkVertex(self):
        
#         pId="l_"+str(self.nLandmarks).zfill(7)
#         self.add_node(pId,t="Landmark",c=self.nLandmarks)
#         self.nLandmarks+=1
#         return pId     
#     def plotLandmark(self,outIMG,landmarkID,setPoses,colour=(0,255,255)):
#         '''
#         setPoses must be in ascending order 000  00001 0002 
#         '''

#         poses=self.getLandmarkConnections(landmarkID)
#         drawn=[]
#         for p in poses:
#             if(p in setPoses):
#                 data=self.edges[p,landmarkID]
#                 pt=(int(data["M"][0,0]),int(data["M"][1,0]))
#                 drawn.append(pt)
#                 if(p==setPoses[-1]):
#                     cv2.circle(outIMG,pt,2,colour,1) 
#                 # else:
#                 #     cv2.circle(outIMG,pt,2,(0,0,255),1) 
#         for d in range(len(drawn)-1):
#             cv2.line(outIMG,drawn[d],drawn[d+1],colour)
   
        
#     def getLandmarkConnections(self,landmarkID):
#         e=self.in_edges(landmarkID)
#         setTracks=[]
#         for a in e:
#             setTracks.append(a[0])
#         return sorted(setTracks)
#     #######################
#     #######################
#     ###calculate Stats
#     ##################

#     ########################################
#     ####Motion Algorithms
#     #########################################
#     def svdRANSAC(self,sourceFrame,targetFrame):
#         print("RANSAC @",targetFrame)
#         trackIDS=self.getLandmarkTracksAT(sourceFrame,targetFrame)


#         maxIterations=100
#         goodModel=0.8*self.nLandmarks
#         bestFit=[[0,0,0,1],[0,0,0]]
#         besterr=np.inf 
#         bestInliers=[]
#         motionRMS=[]
#         minRMSerror=0.1

#         inlierThreshold=0.6

#         count=0
#         minimumInliers=6

#         Xmatrix,Mmatrix=self.getLandmarkTrackDataAT(sourceFrame,targetFrame)
#         count=0
#         msg=PoseStamped()
#         msg.header.frame_id="world"
#         msg.pose.orientation.w=1

#         while(count<maxIterations and besterr>minRMSerror):
#             paramEstimateIndexes,testPointIndexes=randomPartition(trackIDS,3)
#             try:
#                 motionHypothesis=rigid_transform_3D(Xmatrix[0:3,paramEstimateIndexes],
#                                                     Xmatrix[4:7,paramEstimateIndexes])



#                 R=quaternion_matrix(motionHypothesis[0])
#                 T=np.zeros((3,1))
                
#                 T[0,0]=motionHypothesis[1][0]
#                 T[1,0]=motionHypothesis[1][1]
#                 T[2,0]=motionHypothesis[1][2]

#                 H=createHomog(R[0:3,0:3],T)
#                 predictions=getProjections([self.kSettings["Pl"],self.kSettings["Pr"],
#                                             self.kSettings["Pl"].dot(H),self.kSettings["Pr"].dot(H)],
#                             Xmatrix[0:4,testPointIndexes])
#                 e=predictions-Mmatrix[:,testPointIndexes]
#                 inlierIndexes=[]
#                 inlierRMS=[]
#                 for i in range(len(testPointIndexes)):
#                     rms=np.sqrt((e[:,i].flatten()**2).mean())
#                     if(rms<inlierThreshold):
#                         inlierIndexes.append(testPointIndexes[i])
#                         inlierRMS.append(rms)



#                 if(len(inlierIndexes)>len(bestInliers)and(len(inlierIndexes)>minimumInliers)):
                    
#                     ############
#                     ###new model
#                     #plt.plot(inlierRMS)

#                     # betterHypothesis=rigid_transform_3D(Xmatrix[0:3,inlierIndexes],
#                     #                                  Xmatrix[4:7,inlierIndexes])
#                     # R=quaternion_matrix( betterHypothesis[0])
#                     # T=np.zeros((3,1))
                    
#                     # T[0,0]= betterHypothesis[1][0]
#                     # T[1,0]= betterHypothesis[1][1]
#                     # T[2,0]= betterHypothesis[1][2]

#                     H=createHomog(R[0:3,0:3],T)
#                     predictions=getProjections([self.kSettings["Pl"],self.kSettings["Pr"],
#                                                 self.kSettings["Pl"].dot(H),self.kSettings["Pr"].dot(H)],
#                                 Xmatrix[0:4,inlierIndexes])
#                     e=predictions-Mmatrix[:,inlierIndexes]
#                     RMS=np.sqrt((e.flatten()**2).mean())
#                     secondRMS=[]
#                     for i in range(len(inlierIndexes)):
#                         rms=np.sqrt((e[:,i].flatten()**2).mean())
#                         secondRMS.append(rms)
#                     motionRMS=secondRMS
#                     #########
#                     ##convert into transform format
#                     #######
#                     H=np.linalg.inv(H)
#                     q=quaternion_from_matrix(H)
#                     C=H[0:3,0:3].dot(H[0:3,3].reshape(3,1))
#                     cx=C[0,0]
#                     cy=C[1,0]
#                     cz=C[2,0]
#                     betterHypothesis=[q,[cx,cy,cz]]

#                     bestFit=betterHypothesis
#                     besterr=RMS
#                     bestInliers=inlierIndexes 
#                     print("Better Model",besterr,len(bestInliers)) 

#                     #plt.plot(secondRMS,'r')
#                     #plt.show()
                
#             except Exception as e:
#                 print("motion Fail",e)

#             count+=1
#         print("----RESULT",besterr,len(bestInliers))
#         msg.pose.orientation.x=bestFit[0][0]
#         msg.pose.orientation.y=bestFit[0][1]
#         msg.pose.orientation.z=bestFit[0][2]
#         msg.pose.orientation.w=bestFit[0][3]###display is inverse of calculation

#         msg.pose.position.x=bestFit[1][0]
#         msg.pose.position.y=bestFit[1][1]
#         msg.pose.position.z=bestFit[1][2]

#         self.nodes[targetFrame]["msg"].transform.rotation=msg.pose.orientation
#         self.nodes[targetFrame]["msg"].transform.translation.x=msg.pose.position.x
#         self.nodes[targetFrame]["msg"].transform.translation.y=msg.pose.position.y
#         self.nodes[targetFrame]["msg"].transform.translation.z=msg.pose.position.z
#         self.nodes[targetFrame]["inlierRMS"]=motionRMS
#         self.nodes[targetFrame]["inliers"]=[trackIDS[x] for x in bestInliers]#trackIDS[bestInliers]
#         return msg
#     #####################
#     ###query algorithsm
#     #####################
#     def getPoseVertices(self): 
#         s=sorted([x for x in self.nodes() if self.node[x]['t']=="Pose"])
#         return s        
#     def getLandmarkVertices(self):
#         s=sorted([x for x in self.nodes() if self.node[x]['t']=="Landmark"])
#         return s  
#     def getLandmarksVisibleAT(self,poseID):
#         return sorted([x[1] for x in self.edges() if x[0]==poseID])
#     def getLandmarkTracksAT(self,sourcePose,targetPose):
#         tracksSource=self.getLandmarksVisibleAT(sourcePose)
#         tracksTarget=self.getLandmarksVisibleAT(targetPose)
#         return sorted([x for x in tracksSource if x in tracksTarget])
#     def getLandmarkTrackDataAT(self,sourcePose,targetPose):
#         TrackSet=self.getLandmarkTracksAT(sourcePose,targetPose)

#         X=np.zeros((4*2,len(TrackSet)))
#         M=np.zeros((3*2,len(TrackSet)))

#         for k in range(len(TrackSet)):
#             X[0:4,k]=self.edges[(sourcePose,TrackSet[k])]["X"].reshape(4)
#             X[4:8,k]=self.edges[(targetPose,TrackSet[k])]["X"].reshape(4)
#             M[0:3,k]=self.edges[(sourcePose,TrackSet[k])]["M"][0:3,0].reshape(3)
#             M[3:6,k]=self.edges[(targetPose,TrackSet[k])]["M"][0:3,0].reshape(3)
#         return X,M
#     def getLandmarkDataAT(self,setPoses,landmarkID):
#         X =np.zeros((4,len(setPoses)))
#         M=np.zeros((3,len(setPoses)))
        
#         for p in range(len(setPoses)):
#             X[:,p]=self.edges[(setPoses[p],landmarkID)]["X"].reshape(4)
#             M[:,p]=self.edges[(setPoses[p],landmarkID)]["M"][0:3,0].reshape(3)
#         return X,M
#         # R=quaternion_matrix(motionHypothesis[0])
#         # T=np.zeros((3,1))
        
#         # T[0,0]=motionHypothesis[1][0]
#         # T[1,0]=motionHypothesis[1][1]
#         # T[2,0]=motionHypothesis[1][2]

#         # H=createHomog(R[0:3,0:3],T)
#         # predictions=getProjections([self.kSettings["Pl"],self.kSettings["Pr"],
#         #                             self.kSettings["Pl"].dot(H),self.kSettings["Pr"].dot(H)],
#         #             Xmatrix[0:4,testPointIndexes])
#         # e=predictions-Mmatrix[:,testPointIndexes]
#         # inlierIndexes=[]
#         # inlierRMS=[]
#         # for i in range(len(testPointIndexes)):
#         #     rms=np.sqrt((e[:,i].flatten()**2).mean())
#         #     if(rms<1):
#         #         inlierIndexes.append(testPointIndexes[i])
#         #         inlierRMS.append(rms)   

#     def getPoseHomography(self,poseID):
#         msg=self.node[poseID]["msg"]
#         q=[msg.transform.rotation.x,
#                 msg.transform.rotation.y,
#                 msg.transform.rotation.z,
#                 msg.transform.rotation.w]
#         R=quaternion_matrix(q)[0:3,0:3]
#         T=np.array([msg.transform.translation.x,
#                     msg.transform.translation.y,
#                     msg.transform.translation.z]).reshape(3,1)
        
#         return createHomog(R,T)
#     ######################
#     ###display algorithms
#     ######################
#     def publishInlierOutlierTracks(self,sourceFrame,targetFrame,img=None):
#         if(img is None):
#             img=np.zeros((768,1024,3))
#         trackids=self.getLandmarkTracksAT(sourceFrame,targetFrame)
#         for a in trackids:
#             self.plotLandmark(img,a,[sourceFrame,targetFrame],(0,0,255))
#         inlierIds=self.nodes[targetFrame]["inliers"]
#         print(len(inlierIds),len(trackids))
#         for a in inlierIds:
#             self.plotLandmark(img,a,[sourceFrame,targetFrame],(255,255,0))
#         self.debugMatches.publish(self.cvb.cv2_to_imgmsg(img))

#         nInlier=Float32()
#         nInlier.data=len(inlierIds)
#         ratio=Float32()

#         ratio.data=len(inlierIds)/float(len(trackids))
#         self.debugInliers.publish(nInlier)
#         self.debugRatio.publish(ratio)#=rospy.Publisher(displayName+"/debug/ratio",Float32,queue_size=4)
#     def publishPoses(self):
#         p=self.getPoseVertices()
#         for pose in p:
#             m=self.nodes[pose]["msg"]
#             m.header.stamp=rospy.Time()
#             self.debugTF.sendTransformMessage(m)
#     def publishGlobalPoints(self,colour=ColorRGBA(0,0,1,0.5)):
#         l=self.getLandmarkVertices()

#         m=MarkerArray()
#         for k in l:
#             basePose=self.getLandmarkConnections(k)[0]
#             newMarker=Marker()
#             newMarker.header.frame_id=self.displayName+"/"+basePose
#             newMarker.id=self.node[k]["c"]
#             newMarker.type=2
#             newMarker.action=0

#             newMarker.pose.position.x=self.edges[basePose,k]["X"][0,0]
#             newMarker.pose.position.y=self.edges[basePose,k]["X"][1,0]
#             newMarker.pose.position.z=self.edges[basePose,k]["X"][2,0]


#             newMarker.pose.orientation.w=1
#             newMarker.scale.x=0.1
#             newMarker.scale.y=0.1
#             newMarker.scale.z=0.1
#             newMarker.color=colour
#             m.markers.append(newMarker)

#         self.debugMap.publish(m)      
#     def publishLocalPoints(self,PoseID,delta=True):
#         activeLandmarks=self.getLandmarksVisibleAT(PoseID)
#         msg=PointCloud()
#         if(delta):
#             msg.header.frame_id="world"
#         else:
#             msg.header.frame_id=self.displayName+"/"+PoseID
#         msg.header.stamp=rospy.Time()
#         c=ChannelFloat32()
#         c.name="rgb"
#         c.values.append(255)
#         c.values.append(0)
#         c.values.append(0)

#         for landmark in activeLandmarks:
#             inPoint=Point32()
#             inPoint.x=self.edges[PoseID,landmark]["X"][0,0]
#             inPoint.y=self.edges[PoseID,landmark]["X"][1,0]
#             inPoint.z=self.edges[PoseID,landmark]["X"][2,0]
#             msg.points.append(inPoint)
#             msg.channels.append(c)
#         self.debugStereo.publish(msg)
#     def publishCurrentPose(self):
#         activePose=self.getPoseVertices()[-1]

#         msg=PoseStamped()
#         msg.header.frame_id="world"
#         #msg.child_frame_id=self.displayName+"/"+activePose
#         (t,r)=self.listener.lookupTransform('world',self.displayName+"/"+activePose, rospy.Time(0))
        
#         msg.header.stamp=rospy.Time()

#         msg.pose.position.x=t[0]
#         msg.pose.position.y=t[1]
#         msg.pose.position.z=t[2]

#         msg.pose.orientation.x=r[0]
#         msg.pose.orientation.y=r[1]
#         msg.pose.orientation.z=r[2]
#         msg.pose.orientation.w=r[3]

#         self.debugCurrent.publish(msg)
#     def playbackPoses(self,delay=0.2):
#         Poses=self.getPoseVertices()

#         for i in range(1,len(Poses)):
#             print(Poses[i])
#             msg=PoseStamped()
#             msg.header.frame_id="world"
#             #msg.child_frame_id=self.displayName+"/"+activePose
#             (t,r)=self.listener.lookupTransform('world',self.displayName+"/"+Poses[i], rospy.Time(0))
            
#             msg.header.stamp=rospy.Time()

#             msg.pose.position.x=t[0]
#             msg.pose.position.y=t[1]
#             msg.pose.position.z=t[2]

#             msg.pose.orientation.x=r[0]
#             msg.pose.orientation.y=r[1]
#             msg.pose.orientation.z=r[2]
#             msg.pose.orientation.w=r[3]


#             self.debugCurrent.publish(msg)
#             self.publishLocalPoints(Poses[i])

#             img=np.zeros((768,1024,3))



            
#             for k in self.getLandmarkVertices():
#                 self.plotLandmark(img,k,[Poses[i-1],Poses[i]])
#             self.debugMatches.publish(self.cvb.cv2_to_imgmsg(img))
#             time.sleep(delay)

#         print("finished PlayBack")

#     ########################
#     ###graphing opstions
#     def getDeltaMotion(self):
#         Cz=[]
#         Cy=[]
#         Cx=[]

#         roll=[]
#         pitch=[]
#         yaw=[]  
   
#         for p in self.getPoseVertices()[1:]:
#             msg=self.node[p]["msg"]
#             Cx.append(msg.transform.translation.x)
#             Cy.append(msg.transform.translation.y)
#             Cz.append(msg.transform.translation.z)
#             q=[msg.transform.rotation.x,
#                 msg.transform.rotation.y,
#                 msg.transform.rotation.z,
#                 msg.transform.rotation.w]
#             R=euler_from_quaternion(q,'szxy')
#             roll.append(degrees(R[0]))
#             pitch.append(degrees(R[1]))
#             yaw.append(degrees(R[2]))  
#         return roll,pitch,yaw,Cx,Cy,Cz
#     def graphDeltaMotion(self):
#         Rtheta=[]
#         Cz=[]
#         Cy=[]
#         Cx=[]

#         roll=[]
#         pitch=[]
#         yaw=[]
#         for p in self.getPoseVertices()[1:]:
#             msg=self.node[p]["msg"]
#             Cx.append(msg.transform.translation.x)
#             Cy.append(msg.transform.translation.y)
#             Cz.append(msg.transform.translation.z)
#             q=[msg.transform.rotation.x,
#                 msg.transform.rotation.y,
#                 msg.transform.rotation.z,
#                 msg.transform.rotation.w]
#             R=euler_from_quaternion(q,'szxy')
#             roll.append(degrees(R[0]))
#             pitch.append(degrees(R[1]))
#             yaw.append(degrees(R[2]))

#         fig,(ax1,ax2)=plt.subplots(nrows=2, ncols=1,sharex=True,squeeze=True)
#         fig.suptitle(self.displayName)
#         ax1.plot(Cx,label="X")
#         ax1.plot(Cy,label="Y")
#         ax1.plot(Cz,label="Z")
#         ax1.legend()
#         ax2.plot(roll,label="roll")
#         ax2.plot(pitch,label="pitch")
#         ax2.plot(yaw,label="yaw")
#         ax2.legend()


# class slidingWindow(object):
#     def __init__(self,cameraSettings=None,frames=2):
#         if(cameraSettings is None):
#             self.kSettings=None
#         else:
#             self.kSettings=copy.deepcopy(cameraSettings)
#         self.X=np.zeros((0,0),dtype=np.float64) ###[Pose0 Pose1 Pose2|landmarkA landmarkB landmarkC ...]
#         self.M=[]
#         self.tracks=[]
#         self.inliers=[]
#         self.nLandmarks=0
#         self.nPoses=frames
#     ###########
#     ##admin functions
#     ###############
#     def serializeWindow(self):
#         binDiction={}
#         binDiction["kSettings"]=pickle.dumps(self.kSettings)
#         binDiction["M"]=[]
#         for i in self.M:
#             binDiction["M"].append(msgpack.packb(i,default=m.encode))
#         binDiction["X"]=msgpack.packb(self.X,default=m.encode)
#         binDiction["inliers"]=msgpack.dumps(self.inliers)
#         binDiction["tracks"]=msgpack.dumps(self.tracks)
#         binDiction["nLandmarks"]=self.nLandmarks
#         binDiction["nPoses"]=self.nPoses
#         return msgpack.dumps(binDiction)
#     def deserializeWindow(self,data):
#         intern=msgpack.loads(data)
#         self.kSettings=pickle.loads(intern["kSettings"])
#         self.X=msgpack.unpackb(intern["X"],object_hook=m.decode)
#         self.M=[]
#         for i in intern["M"]:
#             self.M.append(msgpack.unpackb(i,object_hook=m.decode))
#         self.inliers=msgpack.loads(intern["inliers"])
#         self.tracks=msgpack.loads(intern["tracks"])
#         self.nLandmarks=intern["nLandmarks"]
#         self.nPoses=intern["nPoses"]
#     ################
#     ##future functions
#     def addLandmark(X,measurements,trackIndexes=None):
#         self.X=np.vstack(X)
#     def addPose(Xpose):
#         pass
#     ##################
#     def getPoseX(self,poseIndex):
#             if(poseIndex==0):
#                 return np.zeros((6,1))
#             else:
#                 realIndex=poseIndex-1
#                 return self.X[6*realIndex:6*realIndex +6,0].reshape(6,1)
#     def getLandmarkX(self,landmarkIndex):
#         return self.X[6 +landmarkIndex*4:6+landmarkIndex*4 +4,0].reshape(4,1)
#     def reprojectLandmark(self,landmarkIndex):
#         '''
#         estimate reprojections from measurement matrix.
#         Triangulations are relative to their respective coordinate frames
#         '''
#         measVect=np.zeros((4,len(self.tracks[landmarkIndex])))
#         measVect[3,:]=np.ones((1,len(self.tracks[landmarkIndex])))
#         measVect[0:2,:]=self.M[landmarkIndex][0:2,:]
#         measVect[2,:]=self.M[landmarkIndex][0,:]-self.M[landmarkIndex][2,:]
#         xreproject=self.kSettings["Q"].dot(measVect)
#         xreproject/=xreproject[3,:]
#         return xreproject
#     def getLandmarkRMS(self,landmarkIndex):

#         #print(np.sum(self.getPixelError(landmarkIndex)**2)))
#         return np.sqrt((self.getPixelError(landmarkIndex)**2).mean())
#     def getAllLandmarkRMS(self):
#         rmsError=[]
#         for i in range(self.nLandmarks):
#             rmsError.append(self.getLandmarkRMS(i))
#         return rmsError
#     def getPixelError(self,landmarkIndex):
#         Mest=np.zeros((4,len(self.tracks[landmarkIndex])))
#         for i in range(0,len(self.tracks[landmarkIndex])):
#             lCam,rCam=self.composeCamera(self.tracks[landmarkIndex][i])
#             lpred=lCam.dot(self.getLandmarkX(landmarkIndex))
#             lpred/=lpred[2,0]
#             Mest[0:2,i]=lpred[0:2,0]
#             rpred=rCam.dot(self.getLandmarkX(landmarkIndex))
#             rpred/=rpred[2,0]
#             Mest[2:4,i]=rpred[0:2,0]
#         # print(Mest)
#         return Mest-self.M[landmarkIndex]
#     def getWindowRMS(self):
#         errorMatrix=[]
#         for landmarkIndex in range(self.nLandmarks):
#             errorMatrix.append(self.getPixelError(landmarkIndex).flatten())
#         eVect=np.concatenate([np.array(i) for i in errorMatrix])
#         return np.sqrt((eVect**2).mean())
#     def composeCamera(self,poseIndex):
#         homog=getH(self.getPoseX(poseIndex))
#         return self.kSettings["Pl"].dot(homog),self.kSettings["Pr"].dot(homog)
#     def getNinliers(self):
#         return np.count(self.inliers)
#     def getSubset(self,landmarkIndexes):

#         subset=slidingWindow(self.kSettings)
#         subset.nLandmarks=len(landmarkIndexes)
#         subset.X=np.zeros((6*(self.nPoses-1)+4*len(landmarkIndexes),1),dtype=np.float64) ###[Pose0 Pose1 Pose2|landmarkA landmarkB landmarkC ...]
#         for i in range(0,self.nPoses-1):
#             subset.X[6*i:6*i+6,0]=self.getPoseX(i+1).reshape(6)
#         for i in range(0,len(landmarkIndexes)):
#             subset.tracks.append(self.tracks[i])
#             subset.X[6 +4*i:6+4*i+4,0]=self.getLandmarkX(landmarkIndexes[i]).reshape(4)
#             subset.M.append(self.M[landmarkIndexes[i]])

#         return subset


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
