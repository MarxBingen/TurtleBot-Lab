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
		red = cv2.bitwise_and(imgC,imgC,mask=maskRed)
		green = cv2.bitwise_and(imgC,imgC,mask=maskGreen)
		red = cv2.cvtColor(red,cv2.COLOR_RGB2GRAY)
		green = cv2.cvtColor(green,cv2.COLOR_RGB2GRAY)
		circlesGreen = cv2.HoughCircles(green,cv2.HOUGH_GRADIENT,1,100,param1=200,param2=100,minRadius=70,maxRadius=380)
		if (circlesGreen is not None):
			print len(circlesGreen)
			circlesGreen = np.uint16(np.around(circlesGreen))
			for i in circlesGreen[0,:]:
				cv2.circle(imgO,(i[0],i[1]),i[2],(0,255,0),2)
		circlesRed = cv2.HoughCircles(red,cv2.HOUGH_GRADIENT,1,100,param1=50,param2=50,minRadius=70,maxRadius=180)
		if (circlesRed is not None):
			print len(circlesRed)
			circlesRed = np.uint16(np.around(circlesRed))
			for i in circles[0,:]:
				cv2.circle(imgO,(i[0],i[1]),i[2],(0,255,0),2)
		#cv2.drawContours(imgO,cntsGreen, -1,(0,255,0),3)
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
