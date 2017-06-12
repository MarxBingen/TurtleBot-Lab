#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False

class test:
	def __init__(self):
		self.bridge = CvBridge()
		#self.template = cv2.imread('kreuz.png',1)
		self.lower_color = np.array([50,200,10], dtype=np.uint8)
		self.upper_color = np.array([70,255,120], dtype=np.uint8)
		self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)
		#self.image_sub = rospy.Subscriber("camera/rgb/image_mono",Image,self.callback)

	def callback(self,data):
		try:
			imgC = self.bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		gray = cv2.cvtColor(imgC, cv2.COLOR_RGB2GRAY)
		#zum filtern nach farbe nach HSV konvertieren
		#hsv = cv2.cvtColor(imgC,cv2.COLOR_RGB2HSV)
		#mask = cv2.medianBlur(hsv,5)
		#mask = cv2.inRange(mask,self.lower_color,self.upper_color)
		#img = cv2.cvtColor(mask,cv2.COLOR_HSV2RGB)
		#img = cv2.cvtColor(mask,cv2.COLOR_RGB2GRAY)
		circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=50,param2=50,minRadius=80,maxRadius=280)
		if (circles is not None):
			print len(circles)
			circles = np.uint16(np.around(circles))
			for i in circles[0,:]:
				cv2.circle(imgC,(i[0],i[1]),i[2],(0,255,0),2)
		cv2.imshow('detected circles',imgC)
		cv2.waitKey(5)

if __name__ == '__main__':
	rospy.init_node('templateMatcher', anonymous=True)
	tm = test()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
