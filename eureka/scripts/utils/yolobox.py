# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 12:22:02 2018

@author: darwin
"""

import sys
import os
import cv2
import copy
import numpy as np
import xml.etree.ElementTree as ET


class BoundBox:
    def __init__(self, xmin, ymin, xmax, ymax, c = None, classes = None):
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        
        self.c     = c
        self.classes = classes

        self.label = -1
        self.score = -1

    def get_label(self):
        if self.label == -1:
            self.label = np.argmax(self.classes)
        
        return self.label
    
    def get_score(self):
        if self.score == -1:
            self.score = self.classes[self.get_label()]
            
        return self.score


def bbox_iou(box1, box2):
    intersect_w = _interval_overlap([box1.xmin, box1.xmax], [box2.xmin, box2.xmax])
    intersect_h = _interval_overlap([box1.ymin, box1.ymax], [box2.ymin, box2.ymax])  
    
    intersect = intersect_w * intersect_h

    w1, h1 = box1.xmax-box1.xmin, box1.ymax-box1.ymin
    w2, h2 = box2.xmax-box2.xmin, box2.ymax-box2.ymin
    
    union = w1*h1 + w2*h2 - intersect
    
    return float(intersect) / union

def _interval_overlap(interval_a, interval_b):
    x1, x2 = interval_a
    x3, x4 = interval_b

    if x3 < x1:
        if x4 < x1:
            return 0
        else:
            return min(x2,x4) - x1
    else:
        if x2 < x3:
             return 0
        else:
            return min(x2,x4) - x3        
        
def draw_boxes(image, boxes, labels):
    image_h, image_w, _ = image.shape

    for box in boxes:
        xmin = int(box.xmin*image_w)
        ymin = int(box.ymin*image_h)
        xmax = int(box.xmax*image_w)
        ymax = int(box.ymax*image_h)
        
        if labels[box.get_label()] == 'r':
            cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (0,0,255), 3)
        if labels[box.get_label()] == 'g':
            cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (0,255,0), 3)
        if labels[box.get_label()] == 'p':
            cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (255,0,0), 3)
        
    return image          
        
def decode_netout(netout, anchors, nb_class, obj_threshold=0.3, nms_threshold=0.3):
    grid_h, grid_w, nb_box = netout.shape[:3]

    boxes = []
    
    # decode the output by the network
    netout[..., 4]  = _sigmoid(netout[..., 4])
    netout[..., 5:] = netout[..., 4][..., np.newaxis] * _softmax(netout[..., 5:])
    netout[..., 5:] *= netout[..., 5:] > obj_threshold
    
    for row in range(grid_h):
        for col in range(grid_w):
            for b in range(nb_box):
                # from 4th element onwards are confidence and class classes
                classes = netout[row,col,b,5:]
                
                if np.sum(classes) > 0:
                    # first 4 elements are x, y, w, and h
                    x, y, w, h = netout[row,col,b,:4]
                    x = (col + _sigmoid(x)) / grid_w # center position, unit: image width
                    y = (row + _sigmoid(y)) / grid_h # center position, unit: image height
                    w = anchors[2 * b + 0] * np.exp(w) / grid_w # unit: image width
                    h = anchors[2 * b + 1] * np.exp(h) / grid_h # unit: image height
                    confidence = netout[row,col,b,4]
                    
                    box = BoundBox(x-w/2, y-h/2, x+w/2, y+h/2, confidence, classes)
                    
                    boxes.append(box)

    # suppress non-maximal boxes
    for c in range(nb_class):
        sorted_indices = list(reversed(np.argsort([box.classes[c] for box in boxes])))

        for i in range(len(sorted_indices)):
            index_i = sorted_indices[i]
            
            if boxes[index_i].classes[c] == 0: 
                continue
            else:
                for j in range(i+1, len(sorted_indices)):
                    index_j = sorted_indices[j]
                    
                    if bbox_iou(boxes[index_i], boxes[index_j]) >= nms_threshold:
                        boxes[index_j].classes[c] = 0
                        
    # remove the boxes which are less likely than a obj_threshold
    boxes = [box for box in boxes if box.get_score() > obj_threshold]
    
    return boxes    


def _sigmoid(x):
    return 1. / (1. + np.exp(-x))

def _softmax(x, axis=-1, t=-100.):
    x = x - np.max(x)
    
    if np.min(x) < t:
        x = x/np.min(x)*t
        
    e_x = np.exp(x)
    
    return e_x / e_x.sum(axis, keepdims=True)



def readData(ann_dir, img_dir, labels_dir,config):
    imgs = []
    targets = []
    seen_labels = {}
    labels = []
    
    anchors = [BoundBox(0, 0, config['ANCHORS'][2*i], config['ANCHORS'][2*i+1]) for i in range(int(len(config['ANCHORS'])//2))]
    
    # get label
    with open(labels_dir,'r') as f:
        for line in f:
            labels.append(line.rstrip())
            
            
    # read img
    for ann in sorted(os.listdir(ann_dir)):
        name = ann[:-4]
        
        
        # read obj
        print('read:',name)
        x = cv2.imread( os.path.join(img_dir,name+".png") )/255.0
        y = np.zeros((config['GRID_H'],  config['GRID_W'], config['BOX'], 4+1+len(labels)))                # desired network output

        
        with open(os.path.join(ann_dir,ann),'r') as f:
            for line in f:
                elem = line.rstrip().split(' ')
                name = elem[4]
                xmin = int(round(float(elem[0])))   
                ymin = int(round(float(elem[1])))   
                xmax = int(round(float(elem[2])))   
                ymax = int(round(float(elem[3])))   
                
                if xmax > xmin and ymax > ymin and name in labels:
                    center_x = .5*(xmin + xmax)
                    center_x = center_x / (float(config['IMAGE_W']) / config['GRID_W'])
                    center_y = .5*(ymin + ymax)
                    center_y = center_y / (float(config['IMAGE_H']) / config['GRID_H'])
                    
                    grid_x = int(np.floor(center_x))
                    grid_y = int(np.floor(center_y))
                  
                    
                    if grid_x < config['GRID_W'] and grid_y < config['GRID_H']:
                        obj_indx  = labels.index(name)
                        
                        center_w = (xmax - xmin) / (float(config['IMAGE_W']) / config['GRID_W']) # unit: grid cell
                        center_h = (ymax - ymin) / (float(config['IMAGE_H']) / config['GRID_H']) # unit: grid cell
                        
                        box = [center_x, center_y, center_w, center_h]

                        # find the anchor that best predicts this box
                        best_anchor = -1
                        max_iou     = -1
                        
                        shifted_box = BoundBox(0,  0, center_w, center_h)
                        
                        for i in range(len(anchors)):
                            anchor = anchors[i]
                            iou    = bbox_iou(shifted_box, anchor)
                            
                            if max_iou < iou:
                                best_anchor = i
                                max_iou     = iou
                                
                        
                        # assign ground truth x, y, w, h, confidence and class probs to y_batch
                        y[ grid_y, grid_x, best_anchor, 0:4] = box
                        y[ grid_y, grid_x, best_anchor, 4  ] = 1.
                        y[ grid_y, grid_x, best_anchor, 5+obj_indx] = 1
                        
            imgs.append(x)
            targets.append(y)
    imgs = np.array(imgs)
    targets = np.array(targets)
    return imgs,targets
                        
        
        
    
"""    
GRID_H,  GRID_W  = 13 , 13
BATCH_SIZE = 1
ANCHORS = [0.57273, 0.677385, 1.87446, 2.06253, 3.33843, 5.47434, 7.88282, 3.52778, 9.77052, 9.16828]
BOX = 5

CLASS_WEIGHTS    = np.ones(3, dtype='float32')
NO_OBJECT_SCALE  = 1.0
OBJECT_SCALE     = 5.0
COORD_SCALE      = 1.0
CLASS_SCALE      = 1.0

BATCH_SIZE       = 16
WARM_UP_BATCHES  = 0
TRUE_BOX_BUFFER  = 50 

#parse_annotation
config =  {
    'IMAGE_H'         : 416, 
    'IMAGE_W'         : 416,
    'GRID_H'          : GRID_H,  
    'GRID_W'          : GRID_W,
    'BOX'             : BOX,
    'ANCHORS'         : ANCHORS,
    'TRUE_BOX_BUFFER' : 50,
}


mdir = os.path.dirname(os.path.abspath(sys.argv[0]))

img_dir = os.path.join(mdir,'Image')
ann_dir = os.path.join(mdir,'Label')
labels_dir = os.path.join(mdir,'class.txt')
imgs,targets = readData(ann_dir, img_dir, labels_dir,config)



"""

















