#!/usr/bin/env python

from TBBase import TBBase
import rospy
from random import randint
import time
import cv2

if __name__ == '__main__':
	base=TBBase(0.35)
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		#print p
		time.sleep(0.1)
