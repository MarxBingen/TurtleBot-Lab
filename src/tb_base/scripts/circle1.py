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
		#Low und High sind in R G B !
		self.redLow = np.array([47,0,0], dtype=np.uint8)
		self.redHigh = np.array([255,16,14], dtype=np.uint8)
		self.greenLow = np.array([0,37,0], dtype=np.uint8)
		self.greenHigh = np.array([12,255,42], dtype=np.uint8)

		self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)
		#self.image_sub = rospy.Subscriber("camera/rgb/image_mono",Image,self.callback)

	def callback(self,data):
		try:
			imgO = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		imgC = cv2.cvtColor(imgO, cv2.COLOR_BGR2RGB)
		imgC = cv2.erode(imgC, None, iterations=2)
		imgC = cv2.dilate(imgC, None, iterations=2)
		#red green masking
		maskRed = cv2.inRange(imgC,self.redLow,self.redHigh)
		maskGreen = cv2.inRange(imgC,self.greenLow,self.greenHigh)
		cntsGreen = cv2.findContours(maskGreen, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		cntsRed = cv2.findContours(maskRed, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
		# only proceed if at least one contour was found
		if len(cntsGreen) > 0:
			c = max(cntsGreen, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			print "Green:", radius
			if radius > 100 and radius < 150:
				cv2.circle(imgO, (int(x), int(y)), int(radius),(0, 255, 0), 2)
		if len(cntsRed) > 0:
			c = max(cntsRed, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			print "Red:", radius
			#if radius > 100 and radius < 150:
			cv2.circle(imgO, (int(x), int(y)), int(radius),(255, 0, 0), 2)
		cv2.drawContours(imgO,cntsGreen, -1,(0,255,0),3)
		cv2.imshow('detected circles',imgO)
		cv2.waitKey(1)

if __name__ == '__main__':
	rospy.init_node('templateMatcher', anonymous=True)
	tm = test()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
