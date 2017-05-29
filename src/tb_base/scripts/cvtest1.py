import cv2 
import numpy as np

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False

class test:

	def run(self):
		while(True):
			#print "Running"
			# Take each frame
			_,img = c.read()
			#img = cv2.flip(img,5)
			img = cv2.medianBlur(img,5)
			img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
			r = self.detect(img)
			cv2.drawContours(img, [r], -1, (0, 255, 0), 2)
			cv2.imshow('detected circles',img)
			cv2.waitKey(5)
		cv2.destroyAllWindows()
		
	def detect(self,gray):
		

		# compute the Scharr gradient magnitude representation of the images
		# in both the x and y direction
		gradX = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 1, dy = 0, ksize = -1)
		gradY = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 0, dy = 1, ksize = -1)

		# subtract the y-gradient from the x-gradient
		gradient = cv2.subtract(gradX, gradY)
		gradient = cv2.convertScaleAbs(gradient)

		# blur and threshold the image
		blurred = cv2.blur(gradient, (9, 9))
		(_, thresh) = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)

		# construct a closing kernel and apply it to the thresholded image
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
		closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

		# perform a series of erosions and dilations
		closed = cv2.erode(closed, None, iterations = 4)
		closed = cv2.dilate(closed, None, iterations = 4)

		# find the contours in the thresholded image
		(_,cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		# if no contours were found, return None
		if len(cnts) == 0:
			return None
		# otherwise, sort the contours by area and compute the rotated
		# bounding box of the largest contour
		c = sorted(cnts, key = cv2.contourArea, reverse = True)[0]
		rect = cv2.minAreaRect(c)
		box = np.int0(cv2.boxPoints(rect))
		# return the bounding box of the barcode
		return box
		
b = test()
b.run()