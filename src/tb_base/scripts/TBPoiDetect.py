#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TBPoiDetect:

	def __init__(self):
		self.bridge = CvBridge()

		self.redLow = np.array([47,0,0], dtype=np.uint8)
		self.redHigh = np.array([255,16,14], dtype=np.uint8)
		self.greenLow = np.array([0,37,0], dtype=np.uint)
		self.greenHigh = np.array([22,255,42], dtype=np.uint)

	def detect(self,data):
		try:
			imgO = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			return "Error"
		imgC = cv2.cvtColor(imgO, cv2.COLOR_BGR2RGB)
		#bild optimierungen
		imgC = cv2.erode(imgC, None, iterations=2)
		imgC = cv2.dilate(imgC, None, iterations=2)
		#red green maskierungen
		maskRed = cv2.inRange(imgC,self.redLow,self.redHigh)
		maskGreen = cv2.inRange(imgC,self.greenLow,self.greenHigh)
		#Konturen finden
		cntsGreen = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		cntsRed = cv2.findContours(maskRed, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		detectedColor = None
		if len(cntsGreen) > 0:
			#die groeste kontur finden
			c = max(cntsGreen, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			# Radius muss passen
			print "Gruen max Radius:", radius
			if radius > 100 and radius < 150:
				detectedColor = "Green"
		if len(cntsRed) > 0:
			c = max(cntsRed, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			print "Rot max Radius:", radius
			if radius > 100 and radius < 150:
				detectedColor = "Red"
		return detectedColor
		
