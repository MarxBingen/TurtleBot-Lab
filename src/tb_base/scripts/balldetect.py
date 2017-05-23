#!/usr/bin/env python
import cv2 
import numpy as np

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False

while(True):

	#print "Running"
	# Take each frame
	_,img = c.read()
	img = cv2.flip(img,5)
	# Convert BGR to HSV
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#cv2.imshow("b", hsv)
	# define range of red color in HSV
	lower_color = np.array([0,100,0], dtype=np.uint8)
	upper_color = np.array([7,255,255], dtype=np.uint8)
	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_color, upper_color)
	cv2.imshow('mask',mask)
	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(img,img, mask= mask)
	#cv2.imshow("b", res)
	imgray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imgray,127,255,0)
	image,contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#erode = cv2.erode(res,None,iterations = 3)
	#dilate = cv2.dilate(erode,None,iterations = 10)
	#contours,hierarchy = cv2.findContours(res,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	for cnt in contours:
		x,y,w,h = cv2.boundingRect(cnt)
		if(w < 20 or h < 20):
			continue
		cx,cy = x+w/2, y+h/2

		#if 100 < res.item(cy,cx,0):
		cv2.rectangle(img,(x,y),(x+w,y+h),[0,0,255],2)
		detected = True
	cv2.imshow('img',img)
	k = cv2.waitKey(5)
	#if k == 27:
	#	break

cv2.destroyAllWindows()
