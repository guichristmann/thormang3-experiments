#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from robotis_controller_msgs.msg import JointCtrlModule
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,String

import sys
import cv2
import time
import os
import numpy as np
from sklearn.cluster import KMeans
import keras
import cv2.aruco as aruco
from keras.models import load_model
from utils import yolobox,dijkstra,turb,engine
#from ik_thormang3.msg import eurekaCmd
from eureka.msg import eurekaCmd
from aruco import camera





## find four corner in a list of points
def findCorner(corners):
    corners = np.reshape(np.array(corners),(4,2))
    rect = np.zeros((4, 2), dtype = "float32")
    s = corners.sum(axis = 1)
  
    rect[0] = corners[np.argmin(s)]
    rect[2] = corners[np.argmax(s)]
    diff = np.diff(corners, axis = 1)
    rect[1] = corners[np.argmin(diff)]
    rect[3] = corners[np.argmax(diff)]
    
    return rect
    
## index for aruco
# 0 1
# 2 3
#_____ tabel
# robot  

## index for Corner
# 0 1
# 3 2
#_____ tabel
# robot    
def findInternalCorner(corners):

    rect = np.zeros((4, 2), dtype = "float32")
    
    ranP = []
    
    for ranC in corners:
        ordC = findCorner(ranC)
        ranP.append(ordC[0])

        
    ordP = findCorner(np.array(ranP))
    
    for i in range(len(ordP)):
        for j in range(len(ranP)):
            if np.array_equal(ordP[i], ranP[j]):
                if i == 0:
                    rect[0] = findCorner(corners[j])[3]
                if i == 1:
                    rect[1] = findCorner(corners[j])[2]  
                if i == 2:
                    rect[2] = findCorner(corners[j])[3] ## switch
                if i == 3:
                    rect[3] = findCorner(corners[j])[2] ## switch
    return rect
    

       
def IDAruco2IDCorner(poseID,ID):
    ans = None
    for i in range(len(poseID)):
        if poseID[i][0] == ID:
            ans = i
            
    return ans
        

class eureka:

    def __init__(self):
    
    
        rospy.init_node('eureka', anonymous=True)
        
        ### Subscriber
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.Imgcallback)
        
        ### Pub
        self.planPub = rospy.Publisher('/thormang3/eureka_controller', eurekaCmd, queue_size=10)


        self.w = 416 ### yolo input image w
        self.h = 416 ### yolo input image h
        
        self.arucoSize  = 0.07
        self.workSpaceW = 0.553#m
        self.workSpaceH = 0.277 - self.arucoSize #m
        

        self.ARw = int(self.workSpaceW * 2000)  # for Perspective image
        self.ARh = int(self.workSpaceH * 2000)  # for Perspective image
       
        
        self.isDetect4Marker = False
        self.corners = None
        self.ids = None
        
        self.arr = np.zeros([480,640,3],dtype=np.uint8)

        ### getPerspectiveTransform dst array
        self.Pdst = np.array([ [0,0],
                               [self.w,0],
                               [self.w,self.h],
                               [0,self.h]],np.float32)
        
        ### image from camera
        self.img = np.zeros((480,640,3), np.uint8)
        ### Perspective image
        self.warped = np.zeros((416,416,3), np.uint8)
        ### AR Perspective image
        self.ARwarped = np.zeros((self.ARw,self.ARh,3), np.uint8)

        ### 
        self.robotStart = False
        name2num = { 
                      
                          'init' : 44,
        }
        self.engine = engine.Engine(name2num)
        
        ###  
        self.turb = turb.Turb()
        self.dijkstra = dijkstra.dijkstra()


        orig = [['red','red'],\
        ['green','green'],\
        ['purple','purple']]



        ### yolo anchors
        self.ANCHORS = [0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]

        ### build yolo model
        self.mdir = os.path.dirname(os.path.abspath(__file__))
        model_path = self.mdir + "/model_data/DrEurela.h5"
        self.model = load_model(model_path)
        self.model.summary()

  
    ### image callback from thormang3 camara
    def Imgcallback(self,img):
       
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        self.arr = np.reshape(arr,(480,640,3))
        

    def run(self):
        
        
        rate = rospy.Rate(10) # 10hz

        
        raw_input("init Pose")
        
        plan = ["init"]
        self.engine.setPlan(plan)
        while self.engine.isRunning:
            self.engine.run()

        raw_input("detect card")

        

        while not rospy.is_shutdown():
            
            #while(not self.turb.isfindball):
                #self.getCard()
                #self.turb.isfindball = True
                #self.turb.symble = [['purple'],\
			    #				    ['green','green','red','red'],\
				#				    ['purple']]

            self.turb.isfindball = True
            if self.turb.isfindball:
                if raw_input("ball is correct?") == "yes":
                    orig = [['red','red'],\
                            ['green','green'],\
                            ['purple','purple']]
                    self.turb.symble = [['red','red','purple'],\
                            ['purple','green'],\
                            ['green']]
                    plan = self.dijkstra.cal_policy(orig,self.turb.symble) 
                    s = eurekaCmd()
                    s.command = plan
                    print(plan)
                    self.planPub.publish(s)
                    break

    def findBall(self,boxes):
        ball = []
        self.turb.clean()
        

        threshold = 20
        ### collection ball from bounding box
        if len(boxes) == 6:
            for box in boxes:
                xmin = int(box.xmin*self.w)
                ymin = int(box.ymin*self.h)
                xmax = int(box.xmax*self.w)
                ymax = int(box.ymax*self.h)
                ball.append([(xmin+xmax)/2,(ymin+ymax)/2,box.get_label()+1])

            ball = np.array(ball)
            ball_x = np.reshape(ball[:,0],[6,1])
            
            ### kmean for 3 clusters
            kmeans = KMeans(n_clusters=3,random_state=0).fit(ball_x)
            c = kmeans.labels_    

            for i in range(len(c)):
                self.turb.add(ball[i].astype(float),c[i])

            ### combine the cluster if have empty turb
            self.turb.combine()
        
            ### sort the turb by position
            self.turb.sort()

        

    def getCard(self):
        rgb = cv2.cvtColor(self.arr, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(self.arr, cv2.COLOR_BGR2GRAY)
        dictionary=aruco.Dictionary_get(aruco.DICT_4X4_1000)
        

        corners, ids, rejectedPoints = aruco.detectMarkers(gray, dictionary, parameters = aruco.DetectorParameters_create() )

        if len(corners) == 4:
            self.isDetect4Marker = True
            for i in ids:
                if i[0] not in [0,1,2,3]:
                    self.isDetect4Marker = False

            self.corners = corners
            self.ids = ids

        if self.isDetect4Marker:
        

            ret,binary = cv2.threshold(gray,120,255,cv2.THRESH_BINARY)
            tmp = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            contours  = tmp[1]
            hierarchy = tmp[0]

            ### iterate all contours to find the contour for the card
            card_contours = []
            minArea = 1000000
            for i in range(len(contours)):
                cnt = contours[i]
                area = cv2.contourArea(cnt)    
                        
                if area > 500 and area < 6000:
                    M = cv2.moments(cnt)
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    isCenter = True
                    for c in self.corners:
                        c_sum = np.sum(c[0],axis = 0)/4
                        
                        if (cx - c_sum[0])**2 + (cy - c_sum[1])**2 < 20000:
                            isCenter = False
                            
                    if isCenter and minArea > area:
                        minArea = area
                        card_contours = [cnt]
                    
            
                     
            ### run yolo model to find the ball in the card
            if len(card_contours) == 1:
                
                epsilon = 20

                pts = cv2.approxPolyDP(card_contours[0],epsilon,True)

                if pts.shape == (4,1,2):   ### check is square
                    #print("area: ",minArea)
                    pts = np.reshape(pts,[4,2])
                
                    rect = np.zeros((4, 2), dtype = "float32")
                    s = pts.sum(axis = 1)
                    rect[0] = pts[np.argmin(s)]
                    rect[2] = pts[np.argmax(s)]
                    diff = np.diff(pts, axis = 1)
                    rect[1] = pts[np.argmin(diff)]
                    rect[3] = pts[np.argmax(diff)]

                    M = cv2.getPerspectiveTransform(rect, self.Pdst)
                    self.warped = cv2.warpPerspective(rgb, M, (416, 416))
                    
                    netout = self.model.predict([[self.warped.copy()/255.0]])
                    boxes = yolobox.decode_netout(netout[0], 
                                                obj_threshold=.05,
                                                nms_threshold=.05,
                                                anchors=self.ANCHORS, 
                                                nb_class=3)
                    self.warped = yolobox.draw_boxes(self.warped, boxes, labels=['r','g','p'])
                    self.findBall(boxes)
            
                    cv2.drawContours(rgb,card_contours,-1,(0,255,0),2)
                    cv2.imshow('perp',self.warped)
                    cv2.waitKey(1)
            
        cv2.imshow('bin',rgb)
        cv2.waitKey(1)
            
     

if __name__ == '__main__':
    e = eureka()
    e.run()
    
    c = camera()
    c.run()
