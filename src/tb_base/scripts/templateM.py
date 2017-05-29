import cv2 
import numpy as np

#cap = cv2.VideoCapture(0)
c = cv2.VideoCapture(0)
detected = False
template = cv2.imread('rot.png',1)
#template = cv2.cvtColor(template,cv2.COLOR_RGB2HSV)
_,w, h = template.shape[::-1]

class test:

	def run(self):
		while(True):
			# Take each frame
			_,img = c.read()
			#img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
			# Apply template Matching
			res = cv2.matchTemplate(img,template,cv2.TM_CCORR_NORMED)
			min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)			
			# If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
			print min_val, max_val
			if min_val > 0.05:
				top_left = max_loc
				bottom_right = (top_left[0] + w, top_left[1] + h)
				cv2.rectangle(img,top_left, bottom_right, 255, 2)
			cv2.imshow('Orig',img)
			#cv2.imshow('Template',res)
			cv2.waitKey(5)
		cv2.destroyAllWindows()
		
b = test()
b.run()