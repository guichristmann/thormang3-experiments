#!/usr/bin/env python

import sys
import numpy as np
import argparse
import cv2
import os
import math
import time

import rospy
from sensor_msgs.msg import Image

from thormang3_archery.msg import target

def applyGaussianBlur(frame, ksize):
    return cv2.GaussianBlur(frame, (ksize, ksize), 0)


def applyEdgeDetector(frame, low, high):
    return cv2.Canny(frame, low, high)


def detectEdges(frame):
    height, width = frame.shape[:2]
    frame = applyGaussianBlur(frame, 7)
    edges = applyEdgeDetector(frame, 100, 200)
    #edges = cv2.resize(edges, (640, 360))
    #print("frame", frame.shape)
    #print("process", frame[0:5, 0:5])
    # processFrame( frame )
    # gray = applyGaussianBlur( frame, 5 )
    return edges


def detectCircles(frame, minRadius, maxRadius, dp, ratio):
    #frame = applyGaussianBlur(frame, 3)
    height, width = frame.shape[:2]
    minAcc = int(2.0 * math.pi * minRadius * ratio)
    circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, dp, 10,
                               param1=30, param2=minAcc, minRadius=minRadius, maxRadius=maxRadius)

    icircles = None
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        icircles = np.round(circles[0, :]).astype("int")

        # loop over the (x, y) coordinates and radius of the circles
        # print("found:", len(circles), circles)
        for (x, y, r) in icircles:
            xt = width * (640 / 1920)
            yt = height * (590 / 1080)

            dist = math.sqrt((x - xt) ** 2 + (y - yt) ** 2)

            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            if dist < 25:
                col = (0,255,0)
            elif (dist < 50 ):
                col = (0, 200, 0)
            elif ( dist < 75 ):
                col = (0, 150, 0)
            elif ( dist < 100):
                col = (0, 100, 0)
            else:
                col = (0,0,0)
            cv2.circle(frame, (x, y), r, col, 1)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    return icircles


def clamp(min, val, max):
    if (val < min):
        res = min
    elif (val > max):
        res = max
    else:
        res = val
    return res

def diffPixel( p1, p2 ):
    d = (p1.astype(int) - p2.astype(int))
    #print(p1, p2, d)
    return d.dot(d)

def maxSuppression( data, rng, threshold ):
    ndata = None
    if ( data is not None ) and (len(data) > 0):
        ndata = np.zeros(len(data))
        for i in range(len(data)):
            if ( data[i] > threshold ):
                ndata[i] = data[i]
                for j in range(-rng,rng+1):
                    if ( i+j >= 0 ) and (i+j) < len(data):
                        if ( data[i] < data[i+j]):
                            ndata[i] = 0
            else:
                data[i] = 0
    return ndata

def findFirstLocalMaxima( data, threshold ):
    up = False
    plateau = 0
    maxIndex = None


    if ( data is not None ) and (len(data) > 0):
        #print("data after max suppression", data)
        prev = data[0]
        for i in range(1, len(data)):
            curr = data[i]

            if ( curr > prev + threshold ):
                up = True
                plateau = i
            elif ( curr < prev - threshold ):
                if (up):
                    maxIndex = int((i + plateau)/2)
                    break
            prev = curr
        return maxIndex

def find3Radius(diffs):
    res = False
    xmax1 = None
    xmax2 = None
    xmax3 = None

    threshold = 100

    diffs = maxSuppression(diffs, len(diffs) // 10, threshold)
    xmax1 = findFirstLocalMaxima(diffs, threshold)
    if xmax1 is not None:
        xmax2 = findFirstLocalMaxima(diffs[xmax1:], threshold)
        if xmax2 is not None:
            xmax3 = findFirstLocalMaxima(diffs[xmax1 + xmax2:], threshold)
            if xmax3 is not None:
                res = (True, xmax1, xmax1 + xmax2, xmax1+xmax2+xmax3)
    return (res, xmax1, xmax2, xmax3)

def getDiffs( frame, start, trans, steps ):
    diffs = np.zeros(steps)
    xc,yc = start
    dx,dy = trans
    prev = frame[yc,xc]
    for i in range( steps ):
        curr = frame[yc,xc]
        diffs[i] = diffPixel(curr, prev)
        xc = xc + dx
        yc = yc + dy
        prev = curr
    return diffs

def checkCircle(frame, circle, rc, trans ):
    res = False, None, None, None
    height, width = frame.shape[:2]
    xc, yc = circle
    xc = int(clamp(0,xc, width-1))
    yc = int(clamp(0,yc, height-1))
    xlim = int(clamp(0, xc + 3.5 * rc * trans[0], width - 1 ))
    ylim = int(clamp(0, yc + 3.5 * rc * trans[1], height - 1 ))

    xdiff = abs(xlim - xc)
    ydiff = abs(ylim - yc)

    if xdiff == 0:
        steps = ydiff
    elif ydiff == 0:
        steps = xdiff
    elif xdiff < ydiff:
        steps = xdiff
    else:
        steps = ydiff

    #print("width", width, "height", height, "xc", xc, "yc", yc, "rc", rc, "trans", trans, "steps", steps, "xlim", xlim, "ylim", ylim, "xdiff", xdiff, "ydiff", ydiff, "xcalc", xc + 3.5 * rc * trans[0], "ycalc", yc + 3.5 * rc * trans[1]  )
    diffs = getDiffs(frame, (xc, yc), trans, steps)
    #print("xc", xc, "yc", yc, "diffs", diffs)
    rad3 = find3Radius(diffs)
    #print("Radius 3", rad3)
    fnd, m1, m2, m3 = rad3
    if fnd:
        r1, r2, r3 = m1, m1 + m2, m1 + m2 + m3
        #print(xc, yc, steps, len(diffs), r1, r2, r3)

        r1avg = (r1 + 0.5 * r2 + 1.0/3.0 * r3)/3.0

        #print("checkCircle", r1avg, r1, r2, r3)
        if (abs(r1 - r1avg) < 0.2 * r1avg):
            if (abs(r2 - 2.0 * r1avg) < 0.2 * r1avg):
                if (abs(r3 - 3.0 * r1avg) < 0.2 * r1avg):
                    res = (True, r1, r2, r3)
                else:
                    #print("Level 2 r3 fail")
                    pass
            else:
                #print("Level 2 r2 fail")
                pass
        else:
            #print("Level 2 r1 fail")
            pass
    else:
        #print("Level 1 fail")
        pass
    return res


def detectTarget(frame):
    qframe = applyGaussianBlur(frame, 3)
    height, width = frame.shape[:2]
    ranges = [ ( 1, 40, 100, 1, 0.3 )]
    
    for rangeIndex in range(len(ranges)):
        scale, minRadius, maxRadius, dp, accRatio = ranges[rangeIndex]
        #print("    *** Range", scale, minRadius, maxRadius, dp, accRatio)
        # edges = detectEdges(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if ( scale > 1 ):
            gray = cv2.resize(gray, (width//scale, height//scale) )
        circles = detectCircles(gray, minRadius, maxRadius, dp, accRatio )
        if (circles is not None):
            for xc, yc, rc in circles:
                
                cv2.circle(gray, (xc, yc), rc, (0, 0, 0), 1)
                #time.sleep(1)
                fndCount = 0
                failedCount = 0
                r1sum = 0
                r2sum = 0
                r3sum = 0
                for t in [ (1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,-1), (1,-1), (-1,1) ]:
                    fnd, r1, r2, r3 = checkCircle(frame, (xc * scale,yc * scale), rc * scale, t )
                    if fnd:
                        #print("Found ray at", t, xc * scale, yc * scale, rc * scale, scale, r1, r2, r3)

                        fndCount = fndCount + 1
                        r1sum = r1sum + r1
                        r2sum = r2sum + r2

                        r3sum = r3sum + r3
                    else:
                        failedCount = failedCount + 1
                        if ( failedCount >= 5):
                            break
                if ( fndCount > 4):
                    #print("Found circle at", fndCount, xc * scale, yc * scale, rc * scale, scale)
                    cv2.circle(frame, (xc * scale, yc * scale), int(((r1sum/fndCount) * scale)/2), (0, 0, 255), 2)
                    cv2.circle(frame, (xc * scale, yc * scale), int(((r2sum/fndCount) * scale)/2), (255, 0, 0), 2)
                    cv2.circle(frame, (xc * scale, yc * scale), int(((r3sum/fndCount) * scale)/2), (255, 255, 255), 2)
                else:
                    #print("Check count failed", fndCount)
                    pass
                delay = False
                
    return None,None,gray


class Vision:

    def __init__(self):
        
        self.img = np.zeros((480,640,3),dtype = np.uint8)
        self.target_pub = rospy.Publisher('/thormang3/archery_target', target, queue_size=10)
        
    def callback(self,img):
        #print("h: " + str(img.height) + " w: " + str(img.width))
        #s = time.time()
        dt = np.dtype(np.uint8)
        dt = dt.newbyteorder('>')
        arr = np.frombuffer(img.data,dtype=dt)
        arr = np.reshape(arr,(480,640,3))
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        self.img = arr
        
        
        #e = time.time()
        #print("time: " + str(e-s))
   
    def run(self):
        
        rospy.init_node('archery_vision', anonymous=True)
        rospy.Subscriber('/robotis/sensor/camera/image_raw', Image, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            rate.sleep()
            frame = self.img
            qframe = applyGaussianBlur(frame, 3)
            height, width = frame.shape[:2]
            ranges = [ ( 1, 40, 100, 1, 0.3 )]
            
            for rangeIndex in range(len(ranges)):
                scale, minRadius, maxRadius, dp, accRatio = ranges[rangeIndex]
                #print("    *** Range", scale, minRadius, maxRadius, dp, accRatio)
                # edges = detectEdges(frame)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if ( scale > 1 ):
                    gray = cv2.resize(gray, (width//scale, height//scale) )
                circles = detectCircles(gray, minRadius, maxRadius, dp, accRatio )
                if (circles is not None):
                    for xc, yc, rc in circles:
                        
                        cv2.circle(gray, (xc, yc), rc, (0, 0, 0), 1)
                        #time.sleep(1)
                        fndCount = 0
                        failedCount = 0
                        r1sum = 0
                        r2sum = 0
                        r3sum = 0
                        for t in [ (1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,-1), (1,-1), (-1,1) ]:
                            fnd, r1, r2, r3 = checkCircle(frame, (xc * scale,yc * scale), rc * scale, t )
                            if fnd:
                                #print("Found ray at", t, xc * scale, yc * scale, rc * scale, scale, r1, r2, r3)

                                fndCount = fndCount + 1
                                r1sum = r1sum + r1
                                r2sum = r2sum + r2

                                r3sum = r3sum + r3
                            else:
                                failedCount = failedCount + 1
                                if ( failedCount >= 5):
                                    break
                        if ( fndCount > 3):
                            #print("Found circle at", fndCount, xc * scale, yc * scale, rc * scale, scale)
                            cv2.circle(frame, (xc * scale, yc * scale), int(((r1sum/fndCount) * scale)/2), (0, 0, 255), 2)
                            cv2.circle(frame, (xc * scale, yc * scale), int(((r2sum/fndCount) * scale)/2), (255, 0, 0), 2)
                            cv2.circle(frame, (xc * scale, yc * scale), int(((r3sum/fndCount) * scale)/2), (255, 255, 255), 2)
                            t = target()
                            t.x = xc
                            t.y = yc
                            self.target_pub.publish(t)
                        else:
                            #print("Check count failed", fndCount)
                            pass
                        delay = False
                
            cv2.imshow('img',self.img)
            cv2.imshow('g',gray)
            cv2.waitKey(1)
        

if __name__ == '__main__':
    v = Vision()
    v.run()
