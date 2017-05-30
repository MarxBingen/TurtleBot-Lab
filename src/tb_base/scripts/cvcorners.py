#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class templateMatcher:

	def __init__(self):
		self.bridge = CvBridge()
		self.template = cv2.imread('rot2.png',1)
		self.lower_color = np.array([160,50,100], dtype=np.uint8)
		self.upper_color = np.array([165,255,200], dtype=np.uint8)
		self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.callback)
		self.detected = False
		self.orb = cv2.ORB_create()
		self.bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)

	def callback(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		#gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		kp1, des1 = self.orb.detectAndCompute(img,None)
		kp2, des2 = self.orb.detectAndCompute(self.template,None)
		matches = self.bf.match(des1,des2)
		matches = sorted(matches,key = lambda x:x.distance)
		img3 = cv2.drawMatches(img,kp1,self.template,kp2,matches[:10],None,flags=2)
		cv2.imshow('img',img3)
		cv2.waitKey(5)

def main(args):
	rospy.init_node('templateMatcher', anonymous=True)
	tm = templateMatcher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
