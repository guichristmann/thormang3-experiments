#!/usr/bin/env python


import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
import time
import os

class camera:

    def __init__(self):
        self.Pdst = np.array([ [0,0],
                               [416,0],
                               [416,416],
                               [0,416]],np.float32)
        self.warped = np.zeros((416,416,3), np.uint8)
        print(os.path.dirname(__file__))
        self.index = 54
        pass
    def callback(self,img):
        #s = time.time()
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        self.getCard(arr)  
        cv2.imshow('img',arr)
        #e = time.time()
        #print("time: " + str(e-s))
        k = cv2.waitKey(0)
        if k == ord('s'):
            print("save: " + str(self.index)) 
            cv2.imwrite(os.path.dirname(__file__) + '/data/' + str(self.index) + '.png',self.warped)
            self.index += 1   
        elif k == ord('n'):
            pass

    def run(self):
        
        rospy.init_node('pyCamera', anonymous=True)
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            rate.sleep()

    def getCard(self,arr):
        img = arr.copy()
        gray = cv2.cvtColor(arr,cv2.COLOR_BGR2GRAY)
        ret,binary = cv2.threshold(gray,140,255,cv2.THRESH_BINARY)
        #edged = cv2.Canny(gray, 15, 200,True)
        tmp = cv2.findContours(binary,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours  = tmp[1]
        hierarchy = tmp[0]
        card_contours = []
        minArea = 1000000
        for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area > 10000 and area < minArea:
                card_contours = [cnt]
                minArea = area


        if len(card_contours) == 1:
            epsilon = 0.1*cv2.arcLength(card_contours[0],True)
            pts = cv2.approxPolyDP(card_contours[0],epsilon,True)
     
            if pts.shape == (4,1,2):
                pts = np.reshape(pts,[4,2])
            
                rect = np.zeros((4, 2), dtype = "float32")
                s = pts.sum(axis = 1)
                rect[0] = pts[np.argmin(s)]
                rect[2] = pts[np.argmax(s)]
                diff = np.diff(pts, axis = 1)
                rect[1] = pts[np.argmin(diff)]
                rect[3] = pts[np.argmax(diff)]

                M = cv2.getPerspectiveTransform(rect, self.Pdst)
                self.warped = cv2.warpPerspective(img, M, (416, 416))
            
        
        cv2.imshow('perp',self.warped)
        cv2.waitKey(1)
        

if __name__ == '__main__':
    c = camera()
    c.run()
