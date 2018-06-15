#!/usr/bin/python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def nothing(x):
    pass

rospy.init_node('mto')

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
msg = Twist()

cap = cv2.VideoCapture(1)

while(1):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    l_y1 = np.array([10, 100, 80])
    u_y1 = np.array([75, 255, 190])
    mask1 = cv2.inRange(hsv, l_y1, u_y1)
    '''l_y2 = np.array([20, 70, 170])
    u_y2 = np.array([40, 170, 255])
    mask2 = cv2.inRange(hsv, l_y2, u_y2)'''
    mask_full = mask1 #+ mask2
    
    st1 = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21), (10, 10))
    st2 = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11), (5, 5))
    #thresh = cv2.morphologyEx(mask_full, cv2.MORPH_CLOSE, st1) 
    thresh = cv2.morphologyEx(mask_full, cv2.MORPH_OPEN, st2) 
    
    blur=cv2.GaussianBlur(thresh,(5,5),2)
    
    _,contours,hierarchy = cv2.findContours(thresh,1,1) 
    
    moments = cv2.moments(blur, 1)
    dM01 = moments['m01']
    dM10 = moments['m10']
    dArea = moments['m00']
    
    if dArea < 1000:
	msg.angular.z = 1
    if dArea > 1000:
        x = int(dM10 / dArea)
        y = int(dM01 / dArea)
        print(x,y)
        cv2.circle(frame, (x, y), 5, (0,0,255), -1)
	if 315<x<325:
	    msg.angular.z = 0
	    msg.linear.x = 0.1
    if dArea > 35000:
	msg.linear.x = 0
    
    pub.publish(msg)
    
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask_full)
    cv2.imshow('blur', blur)

    k = cv2.waitKey(5)
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
