#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#cap = cv2.VideoCapture(0)
#c = cv2.VideoCapture(0)
#detected = False

class test:
	def __init__(self):
		self.bridge = CvBridge()
		#self.template = cv2.imread('kreuz.png',1)
		# define the lower and upper boundaries of the "green"
		self.greenLower = (38, 70, 10)
		self.greenUpper = (95, 255, 96)
		self.redLower = (123,6,0)
		self.redUpper = (181,70,200)
		self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)
		#self.image_sub = rospy.Subscriber("camera/rgb/image_mono",Image,self.callback)

	def callback(self,data):
		try:
			imgC = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		hsv = cv2.cvtColor(imgC, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, self.redLower, self.redUpper)
		#mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			# only proceed if the radius meets a minimum size
			if radius > 100 and radius < 150:
				cv2.circle(imgC, (int(x), int(y)), int(radius),(0, 255, 255), 2)
				cv2.circle(imgC, center, 5, (0, 0, 255), -1)
		cv2.imshow('detected circles',imgC)
		cv2.imshow('mask',mask)
		cv2.waitKey(5)

if __name__ == '__main__':
	rospy.init_node('templateMatcher', anonymous=True)
	tm = test()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
