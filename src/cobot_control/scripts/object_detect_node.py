#!/usr/bin/env python
import sys
import cv2 as cv
import numpy as np
from std_msgs.msg import String
import roslib
import rospy

cap = cv.VideoCapture(0)
pub = rospy.Publisher('error_val',String, queue_size=10)
rospy.init_node('object_detect_node',anonymous=True)
rate = rospy.Rate(10)
#img = cv.imread("4.jpg")
def find_centroid(inp_img):
    img = inp_img
    gray_img = cv.cvtColor(img,cv.COLOR_RGB2GRAY)
    #cv.imshow('BGR',gray_img)

    ret,thresh=cv.threshold(gray_img,60,255,cv.THRESH_BINARY_INV)
    #cv.imshow('Binary Image',thresh)

    kernel = np.ones((5,5), np.uint8)
    filtered = cv.dilate(thresh, kernel)
    #cv.imshow('Filtered',filtered)

    contours, hierarchies = cv.findContours(filtered, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    blank = np.zeros(filtered.shape[:2],dtype='uint8')
    cv.drawContours(blank, contours, -1,(255, 0, 0), 1)
    #cv.imshow("Contours", blank)
    i=0
    for contour in contours:

        approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True)
        
        # using drawContours() function
        cv.drawContours(img, [contour], 0, (0, 0, 255), 5)
    
        # finding center point of shape
        M = cv.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
    
        # putting shape name at center of each shape
        #if len(approx) == 3:
         #   cv.putText(img, 'Triangle', (x, y),
          #              cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        if len(approx) == 4:
            name = 'Workpiece (%d,%d)'%(x,y)
            cv.putText(img, name, (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            pub.publish(name)
    
        #elif len(approx) == 5:
         #   cv.putText(img, 'Pentagon', (x, y),
          #              cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        #elif len(approx) == 6:
           # cv.putText(img, 'Hexagon', (x, y),
            #            cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        #else:
         #   cv.putText(img, 'circle', (x, y),
          #              cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv.imshow('shapes', img)

def start_func():
    while True:
        _,frame = cap.read()
        #cv.imshow('window',frame)
        find_centroid(frame)
        rate.sleep()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    try:
        start_func()
        cap.release()
        cv.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
