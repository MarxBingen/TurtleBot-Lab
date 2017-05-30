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

	def callback(self,data):
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		width,height = self.template.shape[:2]
		#img = img[height/2:height,0:width]
		res = cv2.matchTemplate(img, self.template, cv2.TM_CCOEFF_NORMED)
		min_val,max_val,min_loc,max_loc = cv2.minMaxLoc(res)
		print min_val, max_val
		if max_val > 0.5:
			self.detected = True
		else:
			self.detected = False
		print self.detected
		posX = max_loc[0]+(width/2)
		posY = max_loc[1]+(height/2)
		cv2.circle(img,(posX,posY),20,255,2)
		cv2.imshow('mask',img)
		#cv2.imshow('test',img)
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
