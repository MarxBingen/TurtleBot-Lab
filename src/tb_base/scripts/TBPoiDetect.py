#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TBPoiDetect:

	def __init__(self,imagefile):
		self.bridge = CvBridge()
		self.template = cv2.imread(imagefile,1)

	def detect(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		res = cv2.matchTemplate(img, self.template, cv2.TM_CCOEFF_NORMED)
		min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(res)
		# match level
		print "Match:", max_val
		if max_val > 0.8:
			detected = True
		else:
			detected = False
		return detected
