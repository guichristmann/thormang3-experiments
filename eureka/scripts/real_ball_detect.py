#!/usr/bin/env python


import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import time

class camera:

    def __init__(self):
        self.OriImg = []
    def callback(self,img):
        
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        self.OriImg = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
         
    def findBallRight(self):
    
        if len(self.OriImg) != 0 :
		    
		    
		    hsv = cv2.cvtColor(self.OriImg, cv2.COLOR_BGR2HSV)
		    # define range of red color in HSV 
		    lower_red = np.array([-10,100,100])
		    upper_red = np.array([10,255,255])
		    
		    # Threshold the HSV image to get only red colors   
		    mask_red = cv2.inRange(hsv, lower_red, upper_red)
		    
		    th = 200000
		    num = 0
		    if np.sum(mask_red[364:411,254:286]) > th:
		        num = 1
		        if np.sum(mask_red[310:366,273:308]) > th:
		            num = 2
		            if np.sum(mask_red[263:307,294:325]) > th:
		                num = 3
		                if np.sum(mask_red[203:251,314:348]) > th:
		                    num = 4
		    
		    print(num)
		    
		    
		    cv2.imshow('mask_red',mask_red)
		    
		    cv2.waitKey(1)
		    
		    
    def findBallLeft(self):
    
        if len(self.OriImg) != 0 :
		    
		    
		    hsv = cv2.cvtColor(self.OriImg, cv2.COLOR_BGR2HSV)
		    # define range of red color in HSV 
		    lower_red = np.array([-10,100,100])
		    upper_red = np.array([10,255,255])
		    
		    # Threshold the HSV image to get only red colors   
		    mask_red = cv2.inRange(hsv, lower_red, upper_red)
		    
		    th = 400000
		    num = 0
		    if np.sum(mask_red[342:376,233:297]) > th:
		        num = 1
		        if np.sum(mask_red[264:304,203:272]) > th:
		            num = 2
		            if np.sum(mask_red[192:241,176:254]) > th:
		                num = 3
		                if np.sum(mask_red[110:164,160:231]) > th:
		                    num = 4
		    
		    
		    print(num)
		    
		    
		    cv2.imshow('mask_red',mask_red)
		    
		    cv2.waitKey(1)
   
    def run(self):
        
        rospy.init_node('pyCamera', anonymous=True)
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            rate.sleep()
            self.findBallLeft()
        

if __name__ == '__main__':
    c = camera()
    c.run()
