import cv2 
import numpy as np

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False

class test:

	def run(self):
		while(True):
			_,img = c.read()
			img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
			img = cv2.medianBlur(img,5)
			cimg = img
			circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=50,minRadius=180,maxRadius=200)
			if (circles is not None):
				circles = np.uint16(np.around(circles))
				for i in circles[0,:]:
					cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
			cv2.imshow('detected circles',cimg)
			cv2.waitKey(5)
		cv2.destroyAllWindows()
	
b = test()
b.run()