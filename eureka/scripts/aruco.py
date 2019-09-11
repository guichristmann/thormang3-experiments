#!/usr/bin/env python


import rospy
import numpy as np
from sensor_msgs.msg import Image
from eureka.msg import Position
import os
from sklearn.cluster import KMeans
import cv2
import cv2.aruco as aruco
import time

def plotMarker(img,corners, ids):
    for i in ids:
        Id = i[0]
        corner = np.array(corners[Id][0],dtype=int)
        cv2.polylines(img,[corner],True,(255,255,0),4)

# Converts from camera coordinates to thormang3's coordinates
def tz(tz):
    m = np.eye(4)
    m[2,3]=tz
    return m

def tx(tx):
    m = np.eye(4)
    m[0,3]=tx
    return m

def ry(theta):
    m = np.eye(4)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    m[0,0] = m[2,2] = ctheta
    m[0,2] = -stheta
    m[2,0] = stheta
    return m
    
def from_cam_to_robot(arr,theta):
  num = len(arr)
  pose = np.zeros((num, 4))
  for i in range(num):
      p_cam = np.array([arr[i][0][2],-arr[i][0][0],-arr[i][0][1],1])
      m = np.dot(np.dot(np.dot(tz(1.16),ry(theta)),tz(0.06)),tx(0.052))
      p_robot = np.dot(m,p_cam)
      x, y, z, _ = p_robot
      pose[i] = np.array([x, y, z,1])
  
  ## come from np.linalg.lstsq(a,b,rcond = None)
  M = np.array([[ 9.50391423e-01,  2.25513402e-02,  2.00875685e-15,  9.83547197e-16],
                [-3.84243215e-02,  9.93005669e-01,  8.88178420e-16, -1.11022302e-16],
                [ 1.38203651e+00, -3.54392766e-01, -1.93178806e-14, -1.88737914e-15],
                [-1.19438285e+00,  3.03290003e-01,  9.38500000e-01,  1.00000000e+00]])

  newpose = pose.dot(M)
  #print("newPose: " , newpose)
  return newpose
        
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
                    rect[2] = findCorner(corners[j])[2] ## switch
                if i == 3:
                    rect[3] = findCorner(corners[j])[3] ## switch
    return rect
    

       
def IDAruco2IDCorner(poseID,ID):
    ans = None
    for i in range(len(poseID)):
        if poseID[i][0] == ID:
            ans = i
            
    return ans
            
    
    
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img   

class camera:

    def __init__(self):
        self.circleList = [[],[],[]]
       
        self.circleNum = 1
        
        self.arucoSize  = 0.07  #m
        self.workSpaceW = 0.40  #m
        self.workSpaceH = 0.20 - self.arucoSize #m
        

        self.w = int(self.workSpaceW * 2000)  # for Perspective image
        self.h = int(self.workSpaceH * 2000)  # for Perspective image
        self.aS = int(self.arucoSize * 2000)
        
        self.isDetect4Marker = False
        self.corners = None
        self.ids = None
        
        self.arr = np.zeros([480,640,3],dtype=np.uint8)
        self.mtx = np.load(os.path.dirname(__file__) + "/cameraParameter/cameraMatrix.npy")
        self.dist = np.load(os.path.dirname(__file__) + "/cameraParameter/distCoeffs.npy")
        
        #rospy.init_node('pyCamera', anonymous=True)
        self.pubTubePosition = rospy.Publisher('/thormang3/tube', Position, queue_size=10)
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.callback)

    def callback(self,img):
        #print("h: " + str(img.height) + " w: " + str(img.width))
        #s = time.time()
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        self.arr = np.reshape(arr,(480,640,3))
       
 
    def precess(self):
        isfinish = False
        rgb = cv2.cvtColor(self.arr, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(self.arr, cv2.COLOR_BGR2GRAY)
        
        dictionary=aruco.Dictionary_get(aruco.DICT_4X4_1000)

        corners, ids, rejectedPoints = aruco.detectMarkers(gray, dictionary, parameters = aruco.DetectorParameters_create() )
        ### draw marker
        aruco.drawDetectedMarkers(rgb,corners, ids)
        
      
        
        
        if len(corners) == 4:
           self.isDetect4Marker = True
           for i in ids:
               if i[0] not in [0,1,2,3]:
                    self.isDetect4Marker = False

           self.corners = corners
           self.ids = ids


        if self.isDetect4Marker:

            plotMarker(rgb ,self.corners, self.ids)

            rvecs, tvecs ,_= aruco.estimatePoseSingleMarkers(self.corners,self.arucoSize,self.mtx, self.dist)
            
            Rpose = from_cam_to_robot(tvecs,-np.pi/6) ### pose for robot coor
            PoseID = self.ids
 
            for i in range(rvecs.shape[0]):
                aruco.drawAxis(rgb,self.mtx, self.dist,rvecs[i,:,:], tvecs[i,:,:],0.05)

            
                

            
            ### find Internal Corners
            insidecorners = findInternalCorner(self.corners) 
            
            
            ### draw boundary
            cv2.polylines(rgb,[np.array(insidecorners,dtype=int)],True,(0,255,0),4)
            
            
            Pdst = np.array([[0,0],
                             [self.w,0],
                             [self.w,self.h],
                             [0,self.h]],np.float32)
                             
            
            
            ### detect circle
            hsv = cv2.cvtColor(self.arr, cv2.COLOR_BGR2HSV)
            lower = np.array([0,0,0],np.uint8)
            upper = np.array([180,255,50],np.uint8)
            sep = cv2.inRange(hsv,lower,upper)
            
            kernel = np.ones((9,9),np.float32)/81
            sep = cv2.filter2D(sep,-1,kernel)
            
            M = cv2.getPerspectiveTransform(insidecorners, Pdst)
            warped = cv2.warpPerspective(sep, M, (self.w, self.h))
            warped = 255 - warped
            circles = cv2.HoughCircles(warped,cv2.HOUGH_GRADIENT,1,10,param1=1,param2=30,minRadius=30,maxRadius=50)
                
           
            if circles is not None:
            
                ### remove circle detect the aruco
                new_circles = []
                
                for i in circles[0]:
                    if not (i[1] > self.h - self.aS and i[0] < self.aS):
                        if not (i[1] > self.h - self.aS and i[0] > self.w - self.aS):
                            new_circles.append(i)
                            cv2.circle(warped,(i[0],i[1]),i[2],(0,0,255),2)
                                    
                circles = np.array(new_circles)
                cv2.imshow('gray',warped)
                cv2.waitKey(1)

                if len(circles) >= 3: 
            

                    kmeans = KMeans(n_clusters=3,random_state=0).fit(circles)
                    c = kmeans.labels_ 
                    center = kmeans.cluster_centers_
                        
                    dis0 = np.sqrt((center[0] - center[1]).dot(center[0] - center[1]))
                    dis1 = np.sqrt((center[1] - center[2]).dot(center[1] - center[2]))
                    dis2 = np.sqrt((center[2] - center[0]).dot(center[2] - center[0]))
                    
                    ### check detect three tube in the work space
                    if dis0 > 50 and dis1 > 50 and dis2 > 50:
                        print("three tube")
                        
                        index = np.argsort([center[0][0],center[1][0],center[2][0]])
                        
                        circles = np.uint16(np.around(circles))
                        maxCA = 0
                        maxhA = 0
                        maxCB = 0
                        maxhB = 0
                        maxCC = 0
                        maxhC = 0
                        for i in range(len(circles)):
                            ## left tube
                            if c[i] == index[0]:
                                if circles[i][1] > maxhA:
                                    maxCA = circles[i]
                            ## mid tube
                            if c[i] == index[1]:
                                if circles[i][1] > maxhB:
                                    maxCB = circles[i]          
                            ## right tube
                            if c[i] == index[2]:
                                if circles[i][1] > maxhC:
                                    maxCC = circles[i]
                            
                            
                        
                        self.circleList[0].append(maxCA)
                        self.circleList[1].append(maxCB)
                        self.circleList[2].append(maxCC)
                        
                        L = len(self.circleList[0])
                        if L > 50:
                            warped = cv2.cvtColor(warped, cv2.COLOR_GRAY2RGB)
                            p = Position()
                            tube = np.sum(self.circleList[0][L-10:L-1],axis=0)/9
                            
                            cv2.circle(warped,(tube[0],tube[1]),tube[2],(0,0,255),2)
                            cv2.circle(warped,(tube[0],tube[1]),2,(0,0,255),3)
                            
                            ### translate to Robot coord
                            IDs = IDAruco2IDCorner(PoseID,0)

                            tubeX = Rpose[IDs][0] - (tube[1] * self.workSpaceW/self.w) - self.arucoSize/2 + 0.02
                            tubeY = Rpose[IDs][1] - (tube[0] * self.workSpaceH/self.h) + self.arucoSize/2 
                            tubeZ = Rpose[IDs][2]

                            p.Ax = tubeX
                            p.Ay = tubeY
                            p.Az = tubeZ
                            p.A = True
                            
                            tube = np.sum(self.circleList[1][L-10:L-1],axis=0)/9
                            
                            cv2.circle(warped,(tube[0],tube[1]),tube[2],(0,0,255),2)
                            cv2.circle(warped,(tube[0],tube[1]),2,(0,0,255),3)
                            
                            ### translate to Robot coord
                            IDs = IDAruco2IDCorner(PoseID,0)

                            tubeX = Rpose[IDs][0] - (tube[1] * self.workSpaceW/self.w) - self.arucoSize/2 + 0.02
                            tubeY = Rpose[IDs][1] - (tube[0] * self.workSpaceH/self.h) + self.arucoSize/2 
                            tubeZ = Rpose[IDs][2]
                            
                            p.Bx = tubeX
                            p.By = tubeY
                            p.Bz = tubeZ
                            p.B = True
                            
                            tube = np.sum(self.circleList[2][L-10:L-1],axis=0)/9
                            
                            cv2.circle(warped,(tube[0],tube[1]),tube[2],(0,0,255),2)
                            cv2.circle(warped,(tube[0],tube[1]),2,(0,0,255),3)
                            
                            ### translate to Robot coord
                            IDs = IDAruco2IDCorner(PoseID,0)

                            tubeX = Rpose[IDs][0] - (tube[1] * self.workSpaceW/self.w) - self.arucoSize/2 + 0.02
                            tubeY = Rpose[IDs][1] - (tube[0] * self.workSpaceH/self.h) + self.arucoSize/2 - 0.02
                            tubeZ = Rpose[IDs][2]
                          
                            p.Cx = tubeX
                            p.Cy = tubeY
                            p.Cz = tubeZ
                            p.C = True

                            IDs = IDAruco2IDCorner(PoseID,3)
                            
                            tubeX = Rpose[IDs][0] 
                            tubeY = Rpose[IDs][1] 
                            tubeZ = Rpose[IDs][2]
                            
                            IDs = IDAruco2IDCorner(PoseID,2)
                            
                            tubeX1 = Rpose[IDs][0] 
                            tubeY1 = Rpose[IDs][1] 
                            tubeZ1 = Rpose[IDs][2]
                            
                            p.Dx = (tubeX + tubeX1)/2
                            p.Dy = (tubeY + tubeY1)/2
                            p.Dz = (tubeZ + tubeZ1)/2
                            p.D = True
                            cv2.imshow('gray',warped)
                            cv2.waitKey(1)
                            if raw_input("tube position is correct?") == "yes":
                                self.pubTubePosition.publish(p)
                                #print(os.path.dirname(__file__))
                                isfinish = True
                                #cv2.imwrite(os.path.dirname(__file__) + "/gray.png",warped)
                                #cv2.imwrite(os.path.dirname(__file__) + "/img.png",rgb)
                                    
                        
                        
                        
                    

            
        cv2.imshow('img',rgb)
        cv2.waitKey(1)
        
        return isfinish
        
   
    def run(self):
        # spin() simply keeps python from exiting until this node is stopped
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            isfinish = self.precess()
            if isfinish:
                break
                rate.sleep()
        

if __name__ == '__main__':
    c = camera()
    c.run()
