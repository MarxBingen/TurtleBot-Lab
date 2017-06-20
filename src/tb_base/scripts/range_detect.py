import cv2
import argparse
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from operator import xor

class range_detect:

	def callback(self,value):
		pass

	def setup_trackbars(self,range_filter):
		cv2.namedWindow("Trackbars", 0)
		for i in ["MIN", "MAX"]:
			v = 0 if i == "MIN" else 255
			for j in range_filter:
				cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, self.callback)

	def get_arguments(self):
		ap = argparse.ArgumentParser()
		ap.add_argument('-f', '--filter', required=True,help='Range filter. RGB or HSV')
		ap.add_argument('-i', '--image', required=False,help='Path to the image')
		ap.add_argument('-w', '--webcam', required=False,help='Use webcam', action='store_true')
		ap.add_argument('-p', '--preview', required=False,help='Show a preview of the image after applying the mask',action='store_true')
		args = vars(ap.parse_args())
		if not xor(bool(args['image']), bool(args['webcam'])):
			ap.error("Please specify only one image source")
		if not args['filter'].upper() in ['RGB', 'HSV']:
			ap.error("Please speciy a correct filter.")
		return args

	def get_trackbar_values(self,range_filter):
		values = []
		for i in ["MIN", "MAX"]:
			for j in range_filter:
				v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
				values.append(v)
		return values

	def __init__(self):
		self.args = self.get_arguments()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("camera/rgb/image_color",Image,self.imCallback)
		self.range_filter = self.args['filter'].upper()
		self.setup_trackbars(self.range_filter)


	def imCallback(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
			return
		if self.range_filter == 'RGB':
			frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		else:
			frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values(self.range_filter)
		thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
		if self.args['preview']:
			preview = cv2.bitwise_and(image, image, mask=thresh)
			cv2.imshow("Preview", preview)
		else:
			print "hhuuuhuhu"
			cv2.imshow("Original", image)
			cv2.waitKey(5)
			#cv2.imshow("Thresh", thresh)
		print "Callback-Ende"
		#cv2.waitKey(0)

if __name__ == '__main__':
	rospy.init_node('range_detector', anonymous=True)
	m = range_detect()
	try:
		rospy.spin()
	except:
		print "Ende"
	cv2.destroyAllWindows
