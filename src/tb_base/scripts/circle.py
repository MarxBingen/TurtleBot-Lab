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
		#self.lower_color = np.array([160,50,100], dtype=np.uint8)
		#self.upper_color = np.array([165,255,200], dtype=np.uint8)
		self.image_sub = rospy.Subscriber("camera/rgb/image_mono",Image,self.callback)

	def callback(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)
		#img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
		#img = cv2.medianBlur(img,5)
		img = cv2.GaussianBlur(img,(5,5),0)
		cimg = img
		circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,100,param1=50,param2=50,minRadius=80,maxRadius=180)
		if (circles is not None):
			#print len(circles)
			circles = np.uint16(np.around(circles))
			for i in circles[0,:]:
				cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
		cv2.imshow('detected circles',cimg)
		cv2.waitKey(5)

if __name__ == '__main__':
	rospy.init_node('templateMatcher', anonymous=True)
	tm = test()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
