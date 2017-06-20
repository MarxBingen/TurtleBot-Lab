import cv2 
import numpy as np

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False

class test:

	def run(self):
		lower_color = np.array([50,0,0],dtype=np.uint8)
		upper_color = np.array([70,255,255], dtype=np.uint8)
		while(True):
			_,img = c.read()
			#hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
			imgB = cv2.medianBlur(img,17)
			mask = cv2.inRange(imgB,lower_color,upper_color)
			imgM = cv2.bitwise_and(imgB,imgB, mask = mask)
			grayM = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
			circles = cv2.HoughCircles(grayM,cv2.HOUGH_GRADIENT,1,100,param1=50,param2=50,minRadius=140,maxRadius=180)
			if (circles is not None):
				print len(circles)
				circles = np.uint16(np.around(circles))
				for i in circles[0,:]:
					cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
			#cv2.imshow('Masked',imgM)
			#cv2.imshow('HSV',hsv)
			cv2.imshow('Circles',img)
			cv2.imshow('GRAYM',grayM)
			cv2.waitKey(5)
		cv2.destroyAllWindows()
	
b = test()
b.run()
