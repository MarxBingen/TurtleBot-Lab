#!/usr/bin/env python

from TBBase import TBBase
import rospy
from random import randint
import time
import cv2

if __name__ == '__main__':
	base=TBBase(0.40)
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		if p.rechts == 'Frei':
			base.drehe('rechts')
			base.vorwaerts()
		elif p.mitte == 'Frei':
			base.vorwaerts()
		else:
			base.drehe('links')
		time.sleep(0.5)
